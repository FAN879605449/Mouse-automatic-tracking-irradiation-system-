#Uart������룺
from pynq.overlays.base import BaseOverlay
from pynq.lib import MicroblazeLibrary
import time
base = BaseOverlay('base.bit')
print('finish')

#�򿪴���
lib = MicroblazeLibrary(base.RPI,['uart'])
device = lib.uart_open(14,15)
while True:
#for num in range(10,20):
    if base.buttons[1].read():
        print ('��⵽�����ź� ϵͳֹͣ����')
        break
uart1="X"
uart6="Y"
x,y,w,h=cv2.boundingRect(thresh) #�ҿ�����x��y�����ϵ�����꣬w��h�Ǿ��εı߳���
#Ŀ������ĵ�
true_x=x+w//2
true_y=y+h//2
#���λ�Ʊ���
absolute_x=(true_x-420)
absolute_y=(true_y-235)
#������ת��Ϊʸ���������ַ���
if((5<absolute_x<200)||(5<absolute_y<200)):
    uart2="+"
    uart7="+"
    if(true_x>420):
        uart2="-"
    if(true_y>235):
        uart7="-"
    if(5<absolute_x<200):
        bw_x=absolute_x//100 #��λ
        sw_x=(absolute_x-bw_x*100)//10 #ʮλ
        gw_x=absolute_x%10 #��λ
        uart3="bw_x.getvalue"
        uart4="bw_x.getvalue"
        uart5="bw_x.getvalue"
    else:
        uart3="0"
        uart4="0"
        uart5="0"    
    if(5<absolute_y<200):
        bw_x=absolute_y//100 #��λ
        sw_x=(absolute_y-bw_y*100)//10 #ʮλ
        gw_x=absolute_y%10 #��λ
        uart8="bw_y.getvalue"
        uart9="bw_y.getvalue"
        uart10="bw_y.getvalue"
    else:
        uart8="0"
        uart9="0"
        uart10="0"  
    str1 = [uart1, uart2, uart3, uart4, uart5, uart6, uart7, uart8, uart9, uart10]
    lib.uart_write(device,str1, len(str1))
    time.sleep(1)
    
lib.uart_close(14,15)
