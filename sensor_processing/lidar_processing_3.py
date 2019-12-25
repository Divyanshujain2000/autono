global intrp
def check_if_obstacle_detected(array_radial,dist):
    path = np.concatenate((array_radial[330:360],array_radial[0:30]))
    flag1=0
    for(i in path):    
        if(i < dist):
            flag1=1
            break
    if(flag1 == 1 ):
        return True
    else:
        return False

def check_favourable_path(arr,dist):
    sums=[]
    pos = 0
    global intrp
    sums.append(arr[0])
    for x in range(1,180):
        sums.append(x+sums[-1])
    for i in range(90,170):
        j=sums[i+10]-sums[i]
        j=j/10
        if(j>dist):
            intrp=1
            pos=i+5
            break
    if(intrp == 0):
        for i in range(80,0):
            j=sums[i+10]-sums[i]
            j=j/10
            if(j>dist):
                intrp=1
                pos=i+5
                break
    return pos
    

def call_to_check(rover_rotating_angle,arr,dist):
    flag = 0
    for i in range(rover_rotating_angle-5,rover_rotating_angle+5):
        if(arr[i]<dist):
            flag=1
            break
    if(flag==0):
        return 0
    else:
        return 1


def master_string_generator(fav_path,trigger,straight_allowance,rotation_allowance):
    if(fav_path>0):
        if(fav_path>=rotation_allowance):
            trigger=1
        if(fav_path>=straight_allowance and trigger==1):
            master_string="41703"
        elif(fav_path>straight_allowance and trigger==0):
            master_string="12003"
        else:
            trigger=0
                #print("upper")
            master_string="12003"
    else:
        if(fav_path<=(-1*rotation_allowance)):
            trigger=-1
        if(fav_path<(-1*straight_allowance) and trigger==-1):
            master_string="31703"
        elif(fav_path<=(-1*straight_allowance) and trigger==0):
            master_string="12003"
        else:
            trigger=0
                #print("lower")
            master_string="12003"
    return master_string,trigger


