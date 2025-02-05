from machine import Timer, freq

from PA_SERVO import release

release()
freq(240000000)
import _thread
import padog

t = Timer(1)

c_loop_speed_mode = 0

# /dev/cu.usbserial-14120


def loop(t):
    # LOOP JUDGE
    try:
        global c_loop_speed_mode
        padog.mainloop()
        padog.alarm_run()

        # 航模遥控控制
        if padog.CC_M == 1:
            padog.remote_run()
            padog.height(110)
            padog.X_goal = padog.in_y

        # 串口控制
        elif padog.CC_M == 2:
            padog.UART_Run()
            padog.height(110)
            padog.X_goal = padog.in_y

        # 配置保存后重新初始化循环
        if padog.loop_speed_mode_sc == 1:
            if padog.loop_speed_mode == 0:
                print("Loop mode 1")
                t.deinit()
                t.init(period=5, mode=Timer.PERIODIC, callback=loop)
                padog.loop_speed_mode_sc = 0
            elif padog.loop_speed_mode == 1:
                print("Loop mode 2")
                t.deinit()
                t.init(period=100, mode=Timer.PERIODIC, callback=loop)
                padog.loop_speed_mode_sc = 0
    except:
        t.deinit()
        print("致命错误，重新烧录程序")


def app_1():
    exec(open('web_c.py').read())


t.init(period=5, mode=Timer.PERIODIC, callback=loop)
padog.loop_speed_mode_sc = 0

# time.sleep 时，loop是否会执行？

# 开机启动铃声
padog.start_ring()
# 开启WIFI热点模式，等待连接
padog.do_connect_AP()
# 启动Web服务，用于接收手机控制命令
_thread.stack_size(1024 * 10)
_thread.start_new_thread(app_1, ())
