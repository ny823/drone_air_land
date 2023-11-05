import serial
import yaml

#定义写yaml函数
def write_yaml():
    with open("/home/drone/acfly_ws/src/linetracing/config/mission.yaml", encoding='utf-8',mode='w') as f:
        try:
            yaml.dump(data=response,stream=f,allow_unicode=True)
        except Exception as e:
            print(e)

def init_Uart(Baudrate=9600,dev='/dev/ttyTHS0'):
    ser = serial.Serial(dev, Baudrate)
    if ser.isOpen == False:       
        ser.open()                
    ser.flushInput()
    return ser

#初始化需要接受的参数设置
state = 0
select_1 = 0
select_2 = 0
number = 0
response = {
    "state": state ,
    "po1": select_1,
    "po2": select_2,
    "number": number
}
write_yaml()

#此处可以写保存地点坐标的字典或者数组，直接将坐标传入
Map=[[1,2],
     [3,4],
     [5,6],
     [7,8],
     [9,10],
     [11,12],
     [13,14],
     [15,16],
     [17,18],
     [19,20]]
#初始化串口

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
            write_yaml()
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
                write_yaml()
                print("Mission Set Success!")
                #文件写入完成后继续接收
                accept = ''
            else:
                # 收到的数据不符合格式清空 继续接收
                accept == ''
                continue 
            






