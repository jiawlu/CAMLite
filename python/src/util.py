# @author       Jiawei Lu (jiaweil9@asu.edu)
#               Xuesong Zhou (xzhou74@asu.edu)
# @time         2020/12/21 13:51
# @desc         [script description]

import sys

def getFormattedTime(time_decimal):
    hh = int(time_decimal / 60)
    mm = int(time_decimal - hh*60)
    ss = round((time_decimal - hh*60 - mm)*60,1)
    hh_str = str(hh) if hh >= 10 else '0' + str(hh)
    mm_str = str(mm) if mm >= 10 else '0' + str(mm)
    ss_str = str(ss) if ss >= 10 else '0' + str(ss)
    time_formatted = f'{hh_str}{mm_str}:{ss_str}'
    return time_formatted


def printProgress(current_iter, total_iters, width=30):
    percent = current_iter / total_iters
    show_str = ('[%%-%ds]' % width) % (int(width * percent) * "#")
    print('\r%s %d%%' % (show_str, percent*100), end='')


def stopProgram():
    input()
    sys.exit()