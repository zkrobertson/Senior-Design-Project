import dearpygui.dearpygui as dpg
import pyvisa as visa
import time

    
class servo():
    def __init__(self):
        pass
    
    def refresh(self):
        items = visa.ResourceManager().list_resources()
        dpg.configure_item("servo ports",items=items)

    def connect(self):
        port = dpg.get_value("servo ports")
        try:
            self.device = visa.ResourceManager().open_resource(port,baud_rate=115200)
            self.device.timeout = 5000
            welcome = self.device.read()
            print(welcome)
            if welcome[0] == "W":
                self.connected = True
                dpg.configure_item("servo connection indicator",fill=(0,255,0))
            else:
                print("Connection Error: 02 (Incorrect Serial ID)")
        except:
            print("Connection Error: 01 (Device not Found)")
       
    def disconnect(self):
        try:
            self.device.close()
            self.connected = False
            dpg.configure_item("servo connection indicator",fill=(255,255,0))
        except:
            pass

    def position(self,sender,app_data,user_data):
        position = dpg.get_value("pos")
        print(self.device.query(f"%{position}"))

    def go(self):
        print(self.device.query("move"))

    def summary(self):
        print("?")
        line = self.device.query("?")
        print(line)
        
        while(line[0] != "~"):
            line = self.device.read()
            print(line)
        
class stepper():
    def __init__(self):
        self.connected = False
        self.position = 0
        
        self.grbl_dic = { # initial settings
            '$0':10, # step pulse, usec
            '$1':0, # step idle delay, msec
            '$2':0, # step port invert mask, 00000000
            '$3':3, # dir port invert mask, 00000011
            '$4':1, # step enable invert, bool
            '$5':1, # limit pins invert, bool
            '$6':0, # probe pin invert, bool
            '$10':1, # status report mask:00000011
            '$11':0.010, # junction deviation, mm
            '$12':0.002, # arc tolerance, mm
            '$13':0, # report inches, bool
            '$20':0, # soft limits, bool
            '$21':1, # hard limits, bool
            '$22':1, # homing cycle, bool
            '$23':1, # homing dir invert mask:00000000
            '$24':50.0, # homing feed, mm/min
            '$25':250.0, # homing seek, mm/min
            '$26':250, # homing debounce, msec
            '$27':2.0, # homing pull-off, mm
            '$100':198.0, # x, step/mm
            '$110':2100.0, # x max rate, mm/min
            '$120':10.0, # x accel, mm/sec^2
            '$130':700.0, # x max travel, mm
            }
        
    def refresh(self):
        items = visa.ResourceManager().list_resources()
        dpg.configure_item("stepper ports",items=items)

    def write_grbl_settings(self):
        for key in self.grbl_dic:
            self.write(f"{key}={self.grbl_dic[key]}")
        
    def connect(self):
        port = dpg.get_value("stepper ports")
        try:
            self.device = visa.ResourceManager().open_resource(port,baud_rate=115200)
            
            self.device.write_termination = "\n"  # Setting device communication protocol
            self.device.read_termination = "\r\n"
            self.device.timeout = 5000
            
        except Exception as e: # Failure to connect prints error and returns  
            print(e)
            self.disconnect()
            return
        
        time.sleep(2)   # Wait for grbl to initialize

        try:    # set initial grbl conditions
            self.write_grbl_settings()
        except Exception as e:
            print(e)
            self.disconnect()
            return

        self.set_zero()
        self.connected = True
        dpg.configure_item("stepper connection indicator",fill=(0,255,0))

        #self.pos_thread = threading.Thread(target=self.get_pos)
        #self.pos_thread.start()
        

    def disconnect(self):
        try:
            self.device.close()
            self.connected = False
            dpg.configure_item("stepper connection indicator",fill=(255,255,0))
        except:
            pass

    def get_pos(self):

        while self.connected:
            self.device.write("?")
            line = self.device.read() # <Idle|MPos:0.000,0.000,0.000|FS:0.0,0>
            self.device.read() # ok line
            print(line)

        
    def write(self,str_to_send):
        print(str_to_send)
        if(self.connected):
            self.device.write(str_to_send)
            
    def grbl_manual_command(self,sender,app_data):
        str_to_send = str(app_data)
        self.write(str_to_send)

    def jog(self,sender,app_data,user_data):
        rate = dpg.get_value("rate")
        value = float(user_data)
        
        if sender == "manual jog":
            distance = str(value)
        else:
            distance = str(value)

        if self.position + value <= 750 and self.position + value >= 0:
            str_to_send = f"$J=G91 X{distance} F{rate}"
            self.write(str_to_send)
            self.position += value
        
    def get_configuration(self, config):
        if config == "1":
            position = 0
        elif config == "2":
            position = 132
        elif config == "3":
            position = 280
        elif config == "4":
            position = 400
        elif config == "5":
            position = 555
        elif config == "6":
            position = 687
        else:
            position = 0
            
        return float(position)

    def go_to(self,sender,app_data,user_data):
        rate = dpg.get_value("rate")
        if sender == "go_to_zero":
            position = 0
        elif sender == "go to":
            position = float(app_data)
        elif sender == "preset":
            print("configuration")
            position = self.get_configuration(dpg.get_value("config"))
        else:
            position = 69

        if position > 750:
            position = 750
        elif position < 0:
            position = 0
            
        
        str_to_send = f"$J=G90 X{position} F{rate}"
        self.write(str_to_send)
        self.position = position

    def go_to_zero(self,sender,app_data,user_data):
        rate = dpg.get_value("rate")
        str_to_send = f"$J=X0 F{rate}"
        self.write(str_to_send)

    def set_zero(self):
        str_to_send = "G10 L20 P0 X0"
        self.write(str_to_send)
        self.position = 0

    def stop(self):
        self.device.write("!")

    def home(self):
        self.device.write("$HX")

        
## gui ##
servo = servo()
stepper = stepper()
        
dpg.create_context()
dpg.configure_app(docking=True,docking_space=True)

with dpg.window(label="Servo Control",width=700,height=400):
    dpg.add_combo(items=visa.ResourceManager().list_resources(),width=160,use_internal_label=True,label="COM Ports",tag="servo ports")
    
    dpg.add_button(label="refresh",width=75,callback=servo.refresh)
    dpg.draw_circle(center=[155,56],radius=5,fill=(255,255,0),color=(0,0,0),tag="servo connection indicator")
    with dpg.group(horizontal=True): 
        dpg.add_button(label="connect",callback=servo.connect)
        dpg.add_button(label="disconnect",callback=servo.disconnect)
    with dpg.group(horizontal=True):
        dpg.add_button(label="Set",callback=servo.position)
        dpg.add_input_int(label="position [deg]",width=200,tag="pos")
    with dpg.group(horizontal=True):
        dpg.add_button(label="Go",callback=servo.go)
        dpg.add_button(label="Stop",callback=servo.go)
    dpg.add_button(label="System Summary",callback=servo.summary)

with dpg.window(label="Stepper Control",width=700,height=400):
    dpg.add_combo(items=visa.ResourceManager().list_resources(),width=160,label="COM Ports",tag="stepper ports")
    dpg.add_button(label="refresh",width=75,callback=stepper.refresh)
    
    with dpg.group(horizontal=True):
        dpg.add_button(label="connect",callback=stepper.connect)
        dpg.add_button(label="disconnect",callback=stepper.disconnect)
        
    dpg.draw_circle(center=[155,56],radius=5,fill=(255,255,0),color=(0,0,0),tag="stepper connection indicator")
    dpg.add_text("Stepper Motor Manual Control")
    dpg.add_input_text(label="Jog Rate",tag="rate",default_value=1500,width=264)
    
    with dpg.group(horizontal=True):
        dpg.add_button(label="-10 mm",callback=stepper.jog,user_data="-10")
        dpg.add_button(label="-1 mm",callback=stepper.jog,user_data="-1")
        dpg.add_button(label="1 mm",callback=stepper.jog,user_data="1")
        dpg.add_button(label="10 mm",callback=stepper.jog,user_data="10")
        dpg.add_input_text(label="Manual Jog",callback=stepper.jog,tag="manual jog",on_enter=True,width=60)
        
    with dpg.group(horizontal=True):
        dpg.add_button(label="Go to Zero",callback=stepper.go_to,tag="go_to_zero")
        dpg.add_button(label="Set Zero",callback=stepper.set_zero)
        dpg.add_button(label="Home",callback=stepper.home)
        
    dpg.add_input_text(label="Go To Position",callback=stepper.go_to,on_enter=True,width=60,tag="go to")
    
    with dpg.group(horizontal=True):
        dpg.add_combo(items=["1","2","3","4","5","6"],width=60, label = "Configurations",tag = "config")
        dpg.add_button(label="Go",tag="preset",callback=stepper.go_to)
        
    dpg.add_input_text(label="Manual Grbl Command Input",callback=stepper.grbl_manual_command,on_enter=True,width=200)
    dpg.add_button(label="STOP",callback=stepper.stop)

    

dpg.create_viewport(title="Control Window",width=1000,height=600)
dpg.setup_dearpygui()
dpg.show_viewport()

dpg.start_dearpygui()
dpg.destroy_context()
