"""Receive-only J280 monitor: automatic CSV/raw logging, event marks, 60 s report.

No control bytes are sent to the FPGA. --analyze CSV also works without pyserial.
"""
import argparse
import csv
from collections import deque
from datetime import datetime
import json
import os
from pathlib import Path
import queue
import statistics
import threading
import time
from monitor import Parser, FIELDS, STATES, decode_io_fields, decode_swing_fields, decode_cal_fields

ROOT=Path(__file__).resolve().parents[1]
FAULTS={0:'无故障',1:'采样/健康异常',2:'行程、速度或角速度超限',4:'起摆超时',8:'摆杆倾角超限',16:'计算未及时完成'}


def number(row,key,default=0):
    value=row.get(key)
    return default if value in (None,'') else float(value)


def is_diagnostic(row):
    return bool(int(number(row,'flags'))&32)


def is_r7_jog(row):
    return is_diagnostic(row) and int(number(row,'firmware_tag')) in (0xA407,0xA409,0xA412,0xA413)


def io_readout(row):
    d=decode_io_fields(row)
    if not d:return ''
    def names(mask):return ','.join(f'S{i}' for i in range(4) if mask&(1<<i)) or '无'
    direction={1:'S0正向',-1:'S1反向',0:'未启动'}[d['io_last_direction']]
    return (f"当前原始按键：{names(d['io_raw_keys'])}  消抖确认：{names(d['io_filtered_keys'])}    "
        f"历史见过：{names(d['io_raw_seen'])} / 已确认：{names(d['io_filtered_seen'])}\n"
        f"S0确认 {d['io_s0_presses']} 次   S1确认 {d['io_s1_presses']} 次   点动启动 {d['io_launches']} 次   最近：{direction}\n"
        f"编码器 A={d['encoder_a_level']} B={d['encoder_b_level']}    "
        f"A跳变 {d['encoder_a_edges']}   B跳变 {d['encoder_b_edges']}   非法跳变 {d['encoder_illegal_transitions']}\n"
        f"本次FPGA有效PWM脉冲 {d['motor_pwm_pulses']}    "
        f"{'正在输出' if d['motor_enabled'] else '输出已停'}   AIN1/AIN2={d['motor_ain1_level']}/{d['motor_ain2_level']}（不能代替电机实际转动检查）")


def io_hint(row):
    d=decode_io_fields(row)
    if not d:return ''
    if not d['io_raw_seen']&3:return '尚未收到S0/S1输入；按住S0约0.3秒，松手后记录仍保留'
    if not d['io_filtered_seen']&3:return '收到过原始按键信号，但未通过10ms确认；请按住约0.3秒'
    if not d['io_launches']:return '按键已识别，点动尚未启动；先松开两键，检查页面上的健康或速度提示'
    if not d['motor_pwm_pulses']:return '点动已启动，本次尚未记录有效PWM脉冲；短点按可能尚未形成输出'
    a=d['encoder_a_edges'];b=d['encoder_b_edges']
    if not a and not b:return 'FPGA已发出PWM，尚未看到A/B跳变；结合摆臂是否真的转动检查驱动和编码器反馈'
    if (not a and b>=4) or (not b and a>=4):return '只看到编码器单相多次跳变，请核对另一相信号'
    if d['encoder_illegal_transitions']:return '已记录编码器非法跳变，请结合A/B接线和信号检查'
    delta=int(number(row,'position_counts'))-int(number(row,'travel_origin_counts'))
    if abs(delta)<4:return 'A/B已有跳变，但净位移仍小，可能是来回微动，尚不能确认方向'
    return '已收到编码器反馈；核对S0计数增大、S1计数减小'


def jog_readout(row):
    if not is_r7_jog(row):
        return ''
    position=int(number(row,'position_counts'));origin=int(number(row,'travel_origin_counts'))
    return f"本次点动位移 {position-origin:+d} 计数    累计位置 {position:+d}    已输出 {int(number(row,'capture_ticks'))} ms"


def jog_explanation(row):
    flags=int(number(row,'flags'));fault=int(number(row,'fault'));bits=int(number(row,'limit_mask'))
    parts=['点动模式，无需标定；S0正向、S1反向，松手停，两键同时按也停']
    if flags&8:parts.append('编码器异常，检查后按S4复位')
    if flags&4:parts.append('ADC异常，输出已停止')
    if flags&16:parts.append('ADC采样超时，输出已停止')
    if not is_r7_jog(row):
        parts.append('旧点动固件：固定8%，按住才有输出，最长0.5秒')
        return '；'.join(parts)
    if fault==0x20:parts.append('200ms内未检测到至少4计数位移，已停输出；检查实际转动和编码器反馈')
    elif fault==1:parts.append('采集健康检查未通过，已停输出')
    if bits&1:parts.append('本次达到64计数边界，已停输出')
    if bits&2:parts.append('水平轴速度达到点动上限，已停输出')
    if bits&8:parts.append('本次达到0.5秒，已停输出')
    if bits&32:parts.append('正在点动，力度从8%爬升至20%')
    elif bits&64:parts.append('两键已松开，可以再次点动')
    else:parts.append('请松开两键至少20ms，再按下一次')
    hint=io_hint(row)
    if hint:parts.append(hint)
    return '；'.join(parts)


def explanation(row):
    if is_diagnostic(row):return jog_explanation(row)
    state=int(number(row,'state'));fault=int(number(row,'fault'))
    limits=int(number(row,'limit_mask'));flags=int(number(row,'flags'))
    parts=[]
    if fault: parts.append(FAULTS.get(fault,f'故障{fault:02X}'))
    if limits&1: parts.append('行程已超限，先S1停机检查机构和线缆')
    if limits&2: parts.append('水平轴速度超限')
    if limits&4: parts.append('原始摆杆角速度超限；起摆阶段仅提示')
    if limits&32: parts.append('轨迹等待恢复或到站超时；稳定后拨00再重选')
    if limits&8 and not limits&32: parts.append('倾角偏大，轨迹暂停，平衡仍运行')
    if limits&64: parts.append('自动重试被禁止或次数用完，需人工处理')
    if flags&8: parts.append('编码器异常')
    if flags&64: parts.append('堵转锁存')
    if not flags&1: parts.append('尚未完成标定')
    cal=decode_cal_fields(row)
    if cal.get('down_cal_pending'):
        if cal.get('down_cal_wait_release'):parts.append('下垂标定已申请：松开S3，放开摆杆，让它自然下垂')
        else:parts.append('等待自然下垂稳定1秒后采集；请勿手扶摆杆，S1可取消')
    elif flags&2: parts.append('正在采集标定，保持当前姿态约0.1秒')
    if cal.get('start_inhibited'):
        if flags&1 and abs(number(row,'angle_mdeg'))<165000 and abs(number(row,'angle_mdeg'))>=8000:
            parts.append(f"启动未执行：当前ADC {int(number(row,'adc'))}，下垂标定 {int(number(row,'down_adc'))}；"
                         '若摆杆已自然下垂，请重新标定；软件没有改写标定值')
        else:parts.append('启动未执行：等待采样正常、摆杆及底座稳定后，再按S0')
    if flags&4: parts.append('ADC/OTR异常')
    if flags&16: parts.append('ADC采样超时')
    if flags&128: parts.append('正在人工恢复')
    swing=decode_swing_fields(row)
    if swing:
        phase={0:'初始推摆',1:'跟随摆杆运动',2:'采样恢复等待，暂停推摆'}.get(swing['swing_phase'],'未知阶段')
        parts.append(f"起摆：{phase}；滤后摆速 {swing['swing_filtered_rate_mdeg_ms']}°/秒；"
                     f"当前方向保持 {swing['swing_direction_hold_ms']}ms；拒绝异常样本 {swing['swing_rejected_ticks']}次")
        if int(number(row,'sensor_status'))&8:parts.append('sensor=08为历史角度跳变记录；当前状态看起摆阶段')
    if int(number(row,'version'))==4:
        recovery=int(number(row,'recovery_status'))
        if recovery&1: parts.append('S1停止或恢复锁定，已取消自动重起')
        if recovery&64: parts.append('自动重起后暂用00原点保持；拨动拨码再启用拓展')
        if state==4 and fault in (1,4,8) and not limits&64:
            if fault==1 and not recovery&32: parts.append('此健康故障不允许自动重试')
            elif not recovery&16: parts.append('等待ADC恢复有效')
            elif abs(number(row,'angle_mdeg'))<=160000:
                parts.append('等待摆杆自然下垂；若已下垂仍长期等候，按S1停止后重新标定，重试条件未放宽')
            elif abs(number(row,'angle_delta_mdeg_per_ms'))>=60: parts.append('等待摆杆停止摆动')
            else: parts.append('下垂有效，正在等待重试；重试仍受行程保护')
    elif state==4: parts.append('旧协议未提供重试详情，不能仅凭界面判断停止原因')
    return '；'.join(parts) or ('已实际到位' if limits&16 else '正常运行' if state in (1,2,3) else '待机：S0启动')


class Recorder:
    def __init__(self,directory):
        directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
        stamp=datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.csv_path=directory/f'J280_{stamp}.csv'
        self.start=time.monotonic();self.previous=None;self.gaps=0;self.frames=0
        self.csv_file=self.csv_path.open('w',newline='',encoding='utf-8-sig')
        self.writer=csv.DictWriter(self.csv_file,fieldnames=['received_at','elapsed_s','sequence_gaps_min','telemetry_hz',*FIELDS])
        self.writer.writeheader()
        self.raw_file=self.csv_path.with_suffix('.bin').open('wb')
        self.event_file=self.csv_path.with_suffix('.events.csv').open('w',newline='',encoding='utf-8-sig')
        self.events=csv.writer(self.event_file);self.events.writerow(['received_at','elapsed_s','event'])
        self.lock=threading.Lock();self.closed=False

    def write(self,records,data):
        with self.lock:
            self.raw_file.write(data)
            for r in records:
                seq=r['sequence']
                if self.previous is not None:self.gaps+=(seq-self.previous-1)%256
                self.previous=seq;self.frames+=1
                r.update(received_at=datetime.now().isoformat(timespec='milliseconds'),
                    elapsed_s=round(time.monotonic()-self.start,4),sequence_gaps_min=self.gaps,
                    telemetry_hz=100 if r['version']==4 else 200)
                self.writer.writerow(r)
            self.csv_file.flush();self.raw_file.flush()

    def mark(self,event):
        with self.lock:
            if not self.closed:
                self.events.writerow([datetime.now().isoformat(timespec='milliseconds'),round(time.monotonic()-self.start,4),event])
                self.event_file.flush()

    def close(self):
        with self.lock:
            if self.closed:return
            self.closed=True
            self.csv_file.close();self.raw_file.close();self.event_file.close()


def analyze(path):
    path=Path(path)
    with path.open(encoding='utf-8-sig',newline='') as stream:
        rows=list(csv.DictReader(stream))
    if not rows:raise ValueError('日志中没有有效帧')
    if 'elapsed_s' not in rows[0]:raise ValueError('请用本包监视工具记录CSV，旧日志没有elapsed_s，不能可靠统计60秒')
    if decode_io_fields(rows[-1]):
        summary={'日志':str(path.resolve()),'模式':'ACG720 点动输入输出诊断',
            '帧数':len(rows),'记录跨度_秒':round(number(rows[-1],'elapsed_s')-number(rows[0],'elapsed_s'),3),
            '连续60秒00档平衡记录':False,
            '说明':'点动日志不能验收平衡；累计输入/编码器计数自板上复位开始，PWM计数只对应最近一次启动。',
            '最后位置_计数':int(number(rows[-1],'position_counts')),
            '最后本次位移_计数':int(number(rows[-1],'position_counts')-number(rows[-1],'travel_origin_counts')),
            '记录中故障码':sorted({int(number(r,'fault')) for r in rows}),
            '最后诊断记录':decode_io_fields(rows[-1]),'判断提示':io_hint(rows[-1])}
        target=path.with_suffix('.summary.txt');text=json.dumps(summary,ensure_ascii=False,indent=2)
        target.write_text(text,encoding='utf-8');return target,text
    end=number(rows[-1],'elapsed_s');tail=[r for r in rows if number(r,'elapsed_s')>=end-60]
    duration=number(tail[-1],'elapsed_s')-number(tail[0],'elapsed_s')
    gaps=number(tail[-1],'sequence_gaps_min')-number(tail[0],'sequence_gaps_min')
    intervals=[number(b,'elapsed_s')-number(a,'elapsed_s') for a,b in zip(tail,tail[1:])]
    quiet=all(int(number(r,'state'))==3 and int(number(r,'fault'))==0 and
        (((int(number(r,'keys'))>>4)&3)==0 or int(number(r,'recovery_status'))&64) for r in tail)
    summary={'日志':str(path.resolve()),'统计范围':'最后60秒；不满60秒则统计已有记录',
        '实际跨度_秒':round(duration,3),'帧数':len(tail),'丢帧下限':int(gaps),
        '最大接收间隔_秒':round(max(intervals,default=0),4),
        '连续60秒00档平衡记录':duration>=59.9 and quiet and gaps==0 and max(intervals,default=1)<0.2,
        '说明':'主机接收时间统计；短记录、故障、运动档或丢帧不可当作60秒静止验收。事件记录需核对是否人为推杆。'}
    series={
        '水平对原点误差_计数':[number(r,'position_counts')-number(r,'travel_origin_counts') for r in tail],
        '水平跟踪参考误差_计数':[number(r,'position_counts')-number(r,'target_counts') for r in tail],
        '摆杆角度_度':[number(r,'angle_mdeg')/1000 for r in tail],
        '电机命令_千分比':[number(r,'command_permille') for r in tail]}
    for name,values in series.items():
        summary[name]={'均值':round(statistics.mean(values),4),'峰峰值':round(max(values)-min(values),4),
            '标准差':round(statistics.pstdev(values),4),'最大绝对值':round(max(map(abs,values)),4)}
    text=json.dumps(summary,ensure_ascii=False,indent=2)
    report=path.with_suffix('.summary.txt');report.write_text(text+'\n',encoding='utf-8-sig')
    return report,text


def launch(smoke=False):
    import tkinter as tk
    from tkinter import ttk,messagebox
    root=tk.Tk();root.title('ACG720 r54r13 方向与标定检查 / 自动记录')
    root.geometry(f"1100x{min(940,max(720,root.winfo_screenheight()-100))}")
    q=queue.Queue();stop_event=threading.Event();thread=None;recorder=None;last_path=None;latest=None
    samples=deque(maxlen=1500)
    bar=ttk.Frame(root,padding=10);bar.pack(fill='x')
    port=tk.StringVar();ports=ttk.Combobox(bar,textvariable=port,width=12);ports.pack(side='left')
    status=tk.StringVar(value='先选COM口再连接。串口115200；工具只接收数据，不发送电机命令。')
    pathvar=tk.StringVar(value=str((ROOT/'logs').resolve()))
    details=tk.StringVar(value='未连接')
    info=tk.StringVar(value='等待读数')
    reason=tk.StringVar(value='')
    jogvar=tk.StringVar(value='')
    tracevar=tk.StringVar(value='')

    def refresh():
        try:
            from serial.tools import list_ports
            choices=[p.device for p in list_ports.comports()];ports['values']=choices
            if choices and port.get() not in choices:port.set(choices[0])
        except ImportError:status.set('缺少pyserial：先运行 安装串口依赖.bat')

    def worker(selected,log):
        ser=None;parser=Parser()
        try:
            import serial
            ser=serial.Serial(selected,115200,timeout=0.1)
            q.put(('status','已连接；CSV和原始串口正在自动保存'))
            while not stop_event.is_set():
                data=ser.read(256)
                if not data:continue
                rows=parser.feed(data);log.write(rows,data)
                if rows:q.put(('records',(rows,parser.bad_frames,log.gaps,log.frames)))
        except Exception as exc:q.put(('error',str(exc)))
        finally:
            if ser:ser.close()
            log.close();q.put(('closed',log.csv_path))

    def connect():
        nonlocal thread,recorder,last_path
        if thread and thread.is_alive():return
        if not port.get():messagebox.showerror('COM口','请选择串口');return
        recorder=Recorder(ROOT/'logs');last_path=recorder.csv_path;pathvar.set(str(last_path))
        stop_event.clear();samples.clear()
        thread=threading.Thread(target=worker,args=(port.get(),recorder),daemon=True);thread.start()
        status.set('正在打开串口…')

    def disconnect():
        stop_event.set();status.set('正在断开并保存；点动版松开两键停机，正式版按S1停机。断开串口不控制电机。')

    def report():
        if thread and thread.is_alive():messagebox.showinfo('先断开','先点击断开，CSV完整保存后再生成统计。点动版松开两键停机，正式版按S1停机。');return
        if not last_path:messagebox.showinfo('没有日志','请先连接并记录数据');return
        try:
            target,text=analyze(last_path);messagebox.showinfo('统计已保存',str(target)+'\n\n'+text)
        except Exception as exc:messagebox.showerror('统计失败',str(exc))

    def mark(text):
        if recorder and not recorder.closed:recorder.mark(text);status.set('已记录事件：'+text)
        else:status.set('尚未记录数据：先连接串口，再标记操作事件。')

    ttk.Button(bar,text='刷新串口',command=refresh).pack(side='left',padx=5)
    ttk.Button(bar,text='连接并记录',command=connect).pack(side='left',padx=5)
    ttk.Button(bar,text='断开并保存',command=disconnect).pack(side='left',padx=5)
    ttk.Button(bar,text='打开日志文件夹',command=lambda:os.startfile(str(ROOT/'logs'))).pack(side='left',padx=5)
    ttk.Button(bar,text='生成最后60秒统计',command=report).pack(side='left',padx=5)
    ttk.Label(root,textvariable=status,wraplength=1060,padding=6).pack(anchor='w')
    ttk.Label(root,text='当前日志（连接后自动生成；文件名不会覆盖旧记录）：').pack(anchor='w',padx=10)
    ttk.Entry(root,textvariable=pathvar,state='readonly',width=140).pack(fill='x',padx=10,pady=5)
    ttk.Label(root,textvariable=details,font=('Microsoft YaHei UI',13),padding=10,wraplength=1060).pack(anchor='w')
    ttk.Label(root,textvariable=info,font=('Consolas',12),padding=10,wraplength=1060).pack(anchor='w')
    ttk.Label(root,textvariable=jogvar,font=('Microsoft YaHei UI',18,'bold'),padding=8,wraplength=1060).pack(anchor='w')
    ttk.Label(root,textvariable=tracevar,font=('Microsoft YaHei UI',11),padding=6,wraplength=1060).pack(anchor='w')
    ttk.Label(root,textvariable=reason,font=('Microsoft YaHei UI',11),foreground='#a73516',padding=10,wraplength=1060).pack(anchor='w')
    eventbar=ttk.Frame(root,padding=8);eventbar.pack(fill='x')
    for label in ['标记：静置开始','标记：轻推一下','标记：已松手','标记：故意拨倒','标记：测试结束']:
        ttk.Button(eventbar,text=label,command=lambda v=label:mark(v)).pack(side='left',padx=5)
    canvas=tk.Canvas(root,bg='white',height=160,highlightthickness=1);canvas.pack(fill='both',expand=True,padx=10,pady=5)
    ttk.Label(root,text='图：水平位置（蓝）/参考（橙、绿），单位计数。点动版参考为本次起点；松开两键停机。正式版按S1停机。',padding=8).pack(anchor='w')

    def draw():
        canvas.delete('all')
        if not samples:return
        end=number(samples[-1],'elapsed_s');rows=[s for s in samples if number(s,'elapsed_s')>=end-10]
        w=max(200,canvas.winfo_width());h=max(120,canvas.winfo_height())
        values=[number(r,k) for r in rows for k in ('position_counts','target_counts','final_target_counts') if r.get(k) not in ('',None)]
        lo=min(values)-10;hi=max(values)+10
        canvas.create_text(5,5,anchor='nw',text=f'{hi:.0f}');canvas.create_text(5,h-10,anchor='sw',text=f'{lo:.0f}')
        for key,color in [('position_counts','#2062b4'),('target_counts','#d87b12'),('final_target_counts','#218249')]:
            points=[]
            for r in rows:
                if r.get(key) in ('',None):continue
                x=50+(number(r,'elapsed_s')-(end-10))*(w-65)/10
                y=20+(hi-number(r,key))*(h-40)/(hi-lo);points.extend((x,y))
            if len(points)>=4:canvas.create_line(*points,fill=color,width=2)

    def update():
        nonlocal latest,last_path
        try:
            while True:
                kind,data=q.get_nowait()
                if kind=='records':
                    rows,bad,gaps,count=data;samples.extend(rows);latest=rows[-1]
                    r=latest;v=int(r['version']);tag=f"板上固件 {int(r['firmware_tag']):04X}" if v==4 else '旧协议，无法识别固件版本；请确认烧录工程'
                    mode=(int(number(r,'keys'))>>4)&3
                    mode_text='点动测试（拨码不控制目标）' if is_diagnostic(r) else STATES.get(r['state'],'?')
                    details.set(f"{tag}    {mode_text}  故障 {r['fault']:02X}    拨码(2,1)={mode:02b}\n"
                        f"有效帧 {count}    校验拒绝 {bad}    丢帧下限 {gaps}"+
                        ('    点动记录：松手后保留' if is_diagnostic(r) else f"    重试 {r.get('retry_count','未知')} / 3"))
                    final=r.get('final_target_counts','旧协议未提供')
                    info.set(f"ADC {r['adc']}    标定 UP={r['upright_adc']} DOWN={r['down_adc']}    角度 {r['angle_mdeg']/1000:+.3f}°\n"
                        f"水平位置 {r['position_counts']}    人工原点 {r.get('travel_origin_counts','?')}\n"
                        f"最终目标 {final}    平滑参考 {r['target_counts']}    实际速度 {number(r,'velocity_q8')*1000/256:+.1f} 计数/秒\n"
                        f"PWM命令 {r['command_permille']:+d}‰    limits={int(number(r,'limit_mask')):02X}  flags={r['flags']:02X} sensor={r['sensor_status']:02X}")
                    if is_diagnostic(r):
                        info.set(f"ADC {r['adc']}    无需标定    水平位置 {r['position_counts']:+d}\n"
                            f"PWM命令 {r['command_permille']:+d}‰（{r['command_permille']/10:+.1f}%）    "
                            f"实际速度 {number(r,'velocity_q8')*1000/256:+.1f} 计数/秒\n"
                            f"status={int(number(r,'limit_mask')):02X}  flags={r['flags']:02X}  "
                            f"keys={int(number(r,'keys'))&3:02b}  sensor={r['sensor_status']:02X}")
                    reason.set(explanation(r))
                    jogvar.set(jog_readout(r))
                    tracevar.set(io_readout(r))
                elif kind=='error':status.set('连接/采集失败：'+data)
                elif kind=='status':status.set(data)
                elif kind=='closed':last_path=data;status.set('日志已保存。点动版松开两键停机；正式版按S1停机。')
        except queue.Empty:pass
        draw();root.after(100,update)

    def close():
        stop_event.set()
        if thread and thread.is_alive():root.after(100,close)
        else:root.destroy()
    root.protocol('WM_DELETE_WINDOW',close)
    (ROOT/'logs').mkdir(exist_ok=True);refresh();update()
    if smoke:
        root.withdraw()
        # Exercise the diagnostic layout/update path without a physical COM port.
        row=dict.fromkeys(FIELDS,0)
        row.update(version=4,firmware_tag=0xA413,flags=32,state=0,fault=32,limit_mask=4,
            position_counts=0,travel_origin_counts=0,capture_ticks=200,adc=320,elapsed_s=0.2,
            angle_mdeg=1,integral_permille=0x5300,base_trim_mdeg=1,recovery_status=0x11,
            reference_velocity_q8=7990)
        q.put(('records',([row],0,0,1)))
        swing=row.copy()
        swing.update(flags=1,state=1,fault=0,limit_mask=0,angle_mdeg=179000,
            integral_permille=-37,base_trim_mdeg=1,return_bias_mdeg=60,
            return_damping_mdeg=5,recovery_status=24,elapsed_s=0.4)
        root.after(180,lambda:q.put(('records',([swing],0,0,2))))
        pending=row.copy();pending.update(flags=3,state=0,fault=0,sensor_status=0x70,elapsed_s=0.6)
        root.after(360,lambda:q.put(('records',([pending],0,0,3))))
        root.after(650,close)
    root.mainloop()


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--analyze',type=Path);ap.add_argument('--smoke',action='store_true')
    args=ap.parse_args()
    if args.analyze:print(analyze(args.analyze)[1])
    else:launch(args.smoke)
