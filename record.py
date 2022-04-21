import string
import time
import tkinter as tk
from math import ceil

from pynput import keyboard


class record :
    special_keymap = {
        "Key.enter" : 'ENTER' ,
        "Key.esc" : "ESCAPE" ,
        "Key.backspace" : "BACKSPACE" ,
        "Key.tab" : "TAB" ,
        "Key.space" : "SPACEBAR" ,
        "Key.caps_lock" : "CAPSLOCK" ,
        "Key.print_screen" : "PRINTSCREEN" ,
        "Key.scroll_lock" : "SCROLLLOCK" ,
        "Key.pause" : "PAUSE" ,
        "Key.insert" : "INSERT" ,
        "Key.home" : "HOME" ,
        "Key.page_up" : "PAGEUP" ,
        "Key.delete" : "DELETE" ,
        "Key.end" : "END" ,
        "Key.page_down" : "PAGEDOWN" ,
        "Key.right" : "RIGHTARROW" ,
        "Key.left" : "LEFTARROW" ,
        "Key.down" : "DOWNARROW" ,
        "Key.up" : "UPARROW" ,
        "Key.num_lock" : "NUMLOCK" ,
        "Key.cmd" : "GUI" ,
        "Key.alt_l" : "ALT" ,
        "Key.alt_gr" : "ALT" ,
        "Key.menu" : "KEYBOARDAPPLICATION" ,
        "Key.shift" : "SHIFT" ,
        "Key.shift_r" : "SHIFT" ,
        "Key.ctrl_l" : "CTRL" ,
        "Key.ctrl_r" : "CTRL" ,

    }
    modifier = {
        "Key.cmd" : 0 ,
        "Key.alt_l" : 0 ,
        "Key.alt_gr" : 0 ,
        "Key.shift" : 0 ,
        "Key.shift_r" : 0 ,
        "Key.ctrl_l" : 0 ,
        "Key.ctrl_r" : 0
    }
    time1 = 0
    op = ''
    no_of_modifier = 0
    delay = 00
    modified = 0
    printables = string.punctuation + string.ascii_letters + string.digits
    modifier_key = modifier.keys()
    key_was_aln = 0
    special_keymap_keys = special_keymap.keys()
    time_modifier = 0
    listner = 0
    text = 0
    button = 0
    scroll = 0
    w = 0

    def key_handeler_pressed(self , key) :

        s_key = str( key )
        print( len( s_key ) , key )
        self.delay = ceil( (time.time_ns() - self.time1) / 1000000 )
        if s_key in self.modifier_key :
            self.modified = 0
            self.key_was_aln = 0
            if not self.modifier[s_key] :
                self.no_of_modifier += 1
            self.modifier[s_key] = 1
            # print( s_key , no_of_modifier , modified ,modifier )

        else :
            self.text.config( state='normal' )
            self.modifier_active = 0
            modifier_str = ''
            for x in self.modifier :
                if self.modifier[x] == 1 :
                    self.modifier_active = 1
                    modifier_str = f"{modifier_str}{self.special_keymap[x]}|"
                    self.modified = 1
            modifier_str = modifier_str[:-1]
            # print( modifier_str )
            # print(s_key)
            if s_key in self.special_keymap_keys :
                self.key_was_aln = 0
            if self.time1 != 0 and not self.key_was_aln :
                co = f'''delay({self.delay});'''
                self.op += co
                self.text.insert( tk.INSERT , co + '\n' )
            if (len( s_key ) == 4) & (s_key[0] == "'") :
                # print( modifier_str[:-1] )
                co = f'''keyboard.tapKey('\\\\');'''
                self.op += co
                self.text.insert( tk.INSERT , co + '\n' )
            elif s_key in self.special_keymap_keys :
                if s_key not in self.modifier_key :
                    if self.modifier_active == 1 :
                        co = f'''keyboard.tapSpecialKey(({modifier_str}),{self.special_keymap[s_key]});'''
                        self.op += co
                        self.text.insert( tk.INSERT , co + '\n' )
                    else :
                        co = f'''keyboard.tapSpecialKey({self.special_keymap[s_key]});'''
                        self.op += co
                        self.text.insert( tk.INSERT , co + '\n' )
            elif len( s_key ) == 3 and not (self.modifier["Key.ctrl_r"] == 1 or self.modifier["Key.ctrl_l"] == 1) :
                if s_key[1] in self.printables :
                    if self.modifier_active and not (
                            (self.modifier["Key.shift_r"] == 1 or self.modifier[
                                "Key.shift"] == 1) and self.no_of_modifier == 1) :
                        key_was_aln = 0
                        co = f'''keyboard.tapKey(({modifier_str}),{s_key});'''
                        self.op += co
                        self.text.insert( tk.INSERT , co + '\n' )
                    else :
                        self.key_was_aln = 1
                        co = f'''keyboard.tapKey({s_key});'''
                        self.op += co
                        self.text.insert( tk.INSERT , co + '\n' )
            elif s_key[0 :5] == "Key.f" :
                n = int( s_key[5 :] )
                if self.modifier_active :
                    co = f'''keyboard.tapSpecialKey(({modifier_str}),F{n});'''
                    self.op += co
                    self.text.insert( tk.INSERT , co + '\n' )
                else :
                    co = f'''keyboard.tapSpecialKey(F{n});'''
                    self.op += co
                    self.text.insert( tk.INSERT , co + '\n' )
            elif self.modifier["Key.ctrl_r"] == 1 or self.modifier["Key.ctrl_l"] == 1 :
                O_key = self.listner.canonical( key )
                O_key = str( O_key )
                if O_key[1] in self.printables :
                    self.key_was_aln = 0
                    co = f'''keyboard.tapKey(({modifier_str}),{O_key});'''
                    self.op += co
                    self.text.insert( tk.INSERT , co + '\n' )

            self.time1 = time.time_ns()
            self.text.see( tk.END )
            self.text.config( state='disable' )
            # print( key , no_of_modifier , modified )

    def key_handeler_relased(self , key) :

        self.text.config( state='normal' )
        key = str( key )

        # delay = ceil( (time.time_ns() - time1) / 1000000 )
        if not self.modified :
            if key in self.modifier_key :
                print( self.modified , self.modifier[key] , self.no_of_modifier )
                if self.no_of_modifier == 1 :
                    if self.time1 != 0 :
                        co = f'''delay({self.delay});'''
                        self.op += co
                        self.text.insert( tk.INSERT , co + '\n' )
                    # print()
                    co = f'''keyboard.tapSpecialKey({self.special_keymap[key]});'''
                    print( co )
                    self.op += co
                    self.text.insert( tk.INSERT , co + '\n' )
                    self.modifier[key] = 0
                    self.time1 = time.time_ns()
            self.modified = 1
        if key in self.modifier_key :
            self.no_of_modifier -= 1
            self.modifier[key] = 0
        self.text.see( tk.END )
        self.text.config( state='disable' )

    def button_start(self) :
        if self.button['bg'] == 'green' :
            self.listner = keyboard.Listener( on_press=self.key_handeler_pressed ,
                                              on_release=self.key_handeler_relased )
            self.listner.start()
            self.button['text']='stop'
            self.button['bg'] = 'red'
        else :
            self.on_exit()

    def on_exit(self) :
        try :
            self.listner.stop()
            with open( f"output{self.num}.txt" , "w" )as output :
                output.write( self.op )
        except :
            pass
        self.w.destroy()
        # del self.listner
        # print( key , no_of_modifier , modified )

    def __init__(self , n) :
        self.num = int( n )
        # num=int(input())

        self.w = tk.Tk()
        self.w.geometry( '500x400' )
        self.w.title( f'programming for {self.num}' )
        font = ("Helvetica" , "11")
        self.button = tk.Button( self.w , text='start' , height=2 , bg='green' , width=8 , command=self.button_start )
        self.button.place( x=410 , y=355 )
        self.text = tk.Text( self.w , height=20 , width=58 , font=font )
        self.text.place( x=10 , y=10 )
        self.scroll = tk.Scrollbar( self.w )
        self.scroll.pack( side=tk.RIGHT , fill=tk.Y )
        self.scroll.config( command=self.text.yview )
        self.text.config( yscrollcommand=self.scroll.set )

        self.w.protocol( "WM_DELETE_WINDOW" , self.on_exit )
        self.w.mainloop()
        print( "close me when you are done" )
