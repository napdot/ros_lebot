#!/usr/bin/env python3

import rospy
from st_msgs.msg import String
from lebot.msg import Ref_Command
import json
import websocket as ws
import thread
import asyncio



class Signal:
    def __init__(self):
        self.ref_signals.Publisher('/referee', Ref_Command(), queue_size=5)

    def analyze_command(self, str):
        ref_obj = json.loads(str)
        targets = ref_obj['targets']
        try:    # if target.index returns error, means that we are not on target list and don't need to change anything.
            index = targets.index('LeBot')
            if ref_obj['signal'] == 'start':
                cm = Ref_Command
                cm.command = 'resume'
                self.ref_signals.publish(cm)
                color = refObj['targets'][index]
                rospy.set_param("basket_color", color)
            elif ref_obj['signal'] == 'stop':
                cm = Ref_Command
                cm.command = 'pause'
                self.ref_signals.publish(cm)
            return
        except:
            return


async def execute():
    global msg_ws, new_msg
    ws_addr = 'ws://127.0.0.1:9000'
    new_msg = False
    async with websockets.connect(ws_addr) as websocket:
        while True:
            msg_ws = await websocket.recv()
            if not new_msg:
                new_msg = True


def init_ws():
    asyncio.get_event_loop().run_until_complete(execute())


if __name__ == '__main__':
    new_msg = False
    rospy.init_node('ref', anonymous=False)
    ref = Signal()
    t = Thread(target=init_ws, daemon=True)
    t.start
    while not rospy.is_shutdown():
        if new_msg:
            ref.analyze_command(msg_ws)
            new_msg = False
