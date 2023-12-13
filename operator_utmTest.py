from Operator import Operator
from UTM import UTM
from config import DEFAULT_DRONE1, DEFAULT_DRONE2
import sys

DRONE02 = DEFAULT_DRONE1
DRONE04 = DEFAULT_DRONE2
if len(sys.argv) > 1:
    DRONE02 = sys.argv[1]
    DRONE04 = sys.argv[2]

utm = UTM()
op02 = Operator(DRONE02, utm)
op04 = Operator(DRONE04, utm)
utm.operators = [op02, op04]

op02.send_mission([['office', 2, 0.815], ['helibase', 0, 0]])
op04.send_mission([['office', 2, 0.815], ['helibase', 0, 0]])

utm.fly_all()
print(utm.reserved)