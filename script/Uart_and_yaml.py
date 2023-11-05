import serial
import yaml
import subprocess
import rospy

#发送特定字符串给单片机，激发蜂鸣器
def buzzer():
    print("do sth")

#定义写yaml函数
def write_yaml(responses):
    with open("/home/drone/acfly_ws/src/linetracing/config/mission.yaml", encoding='utf-8',mode='w') as f:
        try:
            yaml.dump(data=responses,stream=f,allow_unicode=True)
        except Exception as e:
            print(e)
def read_yaml():
    with open("/home/drone/acfly_ws/src/linetracing/config/mission.yaml",encoding="utf-8") as f:
            data = yaml.load(f, yaml.FullLoader)
            print(data)
            return data

def init_Uart(Baudrate=9600,dev='/dev/ttyTHS0'):
    subprocess.call('echo "123456" | sudo -S chmod 777 /dev/ttyTHS0', shell=True)
    ser = serial.Serial(dev, Baudrate)
    if ser.isOpen == False:       
        ser.open()                
    ser.flushInput()
    return ser

#重置yaml文件信息
def clean_yaml():
    response = {
    "state": 0 ,
    "po1": 0,
    "po2": 0,
    "number": 0,
    "begin" : 0
    }
    with open("/home/drone/acfly_ws/src/linetracing/config/mission.yaml", encoding='utf-8',mode='w') as f:
        try:
            yaml.dump(data=response,stream=f,allow_unicode=True)
        except Exception as e:
            print(e)

def start_uart():
    #可写一个10hz左右循环 循环一次读取一次ros参数 若参数改变 根据改变情况 激活蜂鸣器

    rospy.set_param("/mission_param",[0.0,0.0,0.0,0.0])
    #初始化参数
    state = 0
    select_1 = 0
    select_2 = 0
    number = 0
    begin = 0
    response = {
        "state": state ,
        "po1": select_1,
        "po2": select_2,
        "number": number,
        "begin" : begin
    }
    clean_yaml()
    ser = init_Uart(Baudrate=9600,dev='/dev/ttyTHS0')
    accept = ''
    while 1:
        while accept == '':
            if(accept == ''):
                print('waiting...')
            else:
                print('erorr!!!')
            accept = ser.read(6)
            # accept = ser.readline()
            accept = accept.decode("utf-8")
            print(accept)
            if(accept[0]=='a' and accept[1]=='c' and accept[5] == 'z'):
                state = int(accept[2])
                select_1 = int(accept[3])
                select_2 = int(accept[4])
                number = 0
                print("var set success!")
                #要写入的内容
                response = {
                    "state": state ,
                    "po1": select_1,
                    "po2": select_2,
                    "number": number
                }
                write_yaml(response)
                print("Mission Set Success!")
                #文件写入完成后继续接收
                accept = ''
            else :
                if(accept[0]=='a' and accept[1]=='d' and accept[5] =='z'):
                    state = int(accept[2])
                    select_1 = 0
                    select_2 = 0
                    number = 0
                    print("var set success!")
                    #要写入的内容
                    response = {
                        "state": state ,
                        "po1": select_1,
                        "po2": select_2,
                        "number": number
                    }
                    write_yaml(response)
                    print("Mission Set Success!")
                    #文件写入完成后继续接收
                    accept = ''
                else:
                    # 收到的数据不符合格式清空 继续接收
                    accept == ''
                    continue 
            






