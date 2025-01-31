from udpcanpy import NetworkHandler, RemoteControl

nh = NetworkHandler()
res = nh.parse("src/sim_mdrs/sim_mdrs/comms.dbc")
if(res != 0):
    print(f"Parse {res}")

res = nh.init()
if(res != 0):
    print(f"Init {res}")

res = nh.start()
if(res != 0):
    print(f"Start {res}")

remote = nh.getRemoteControl()

data = RemoteControl()

while True:
    res = remote.access(data)
    if res == 0:
        print(f"=================\n\
              LB: {data.l_bottom}\n\
              LT: {data.l_top}\n\
              LR: {data.l_right}\n\
              LL: {data.l_left}\n\
              RB: {data.r_bottom}\n\
              RT: {data.r_top}\n\
              RR: {data.r_right}\n\
              RL: {data.r_left}\n\
              LS: {data.l_shoulder}\n\
              RS: {data.r_shoulder}\n\
              LTrigger: {data.left_trigger}\n\
              RTrigger: {data.right_trigger}\n\
              ThumbLX: {data.thumb_left_x}\n\
              ThumbLY: {data.thumb_left_y}\n\
              ThumbRX: {data.thumb_right_x}\n\
              ThumbRY: {data.thumb_right_y}\n\
                ")

    if data.e_stop:
        break

res = nh.stop()
if(res != 0):
    print(f"Stop {res}")

print("Adiós")