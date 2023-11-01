import matplotlib.pyplot as plt
import re

def HHMM_to_mins(HHMM):
    parts = HHMM.split(":")
    return int(parts[0])*60 + int(parts[1])

def mins_to_HHMM(mins):
    return "%.2d:%.2d" %(int(mins/60), mins%60)

def readSchedule(file):
    sched = {}
    with open(file,"r") as f:
        for line in f:
            parts = re.split('[\t \-\n]', line.strip("\n"))
            if (len(parts) == 1): pass
            elif (len(parts) != 3):
                raise Exception("Syntax error reading schedule: %s" %line)
            else:
                behaviorname = parts[0]
                if (not behaviorname in sched): sched[behaviorname] = []

                endl = parts[2].split(":")
                sched[behaviorname].append((HHMM_to_mins(parts[1]),
                                            HHMM_to_mins(parts[2])))
        return sched

def writeSchedule(file, schedule):
    with open(file,"w") as f:
        for behavior in sorted(schedule):
            for times in schedule[behavior]:
                f.write("%s %s-%s\n" %(behavior, mins_to_HHMM(times[0]),
                                    mins_to_HHMM(times[1])))
            f.write("\n")


def displaySchedule(schedule):
    # Declaring a figure "gnt"
    fig, gnt = plt.subplots()
    fig.set_size_inches(12, max(1, 0.5*len(schedule)))

    # Setting axes limits
    gnt.set_xlim(0, 24*60)
    gnt.set_ylim(0, 3*len(schedule)+1)

    # Setting axes labels
    gnt.set_xlabel('Hours Since Midnight')
    gnt.set_ylabel('Activity')

    # Setting axes ticks and labels
    gnt.set_xticks([60*i for i in range(0,25)])
    gnt.set_xticklabels([i for i in range(0,25)])

    behaviors = sorted(schedule, reverse=True)
    gnt.set_yticks([3*i-1 for i in range(1,len(schedule)+1)])
    gnt.set_yticklabels(behaviors)

    # Setting graph attribute
    gnt.grid(True)

    colors = ('blue', 'red', 'orange', 'green', 'yellow')
    index = 0
    for behavior in behaviors:
        bars = [(t[0], t[1]-t[0]) for t in schedule[behavior]]
        print(bars, index, colors[index%len(colors)])
        gnt.broken_barh(bars, [(3*index+1), 2],
                        facecolors=colors[index%len(colors)],
                        edgecolor='black')
        index += 1

    plt.tight_layout()
    plt.ion()
    plt.show()