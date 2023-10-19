import multiprocessing as mp
import time
from multiprocessing.managers import DictProxy

N_THREADS = mp.cpu_count()

def print_useless(E):
    for i in range(10):
        if E.is_set():
            break
        print("Driving")
        time.sleep(1)

def print_useless_2():
    for i in range(10):
        print("Driving 2")
        time.sleep(1)


if __name__ == '__main__':

    E = mp.Event()
    p1 = mp.Process(target = print_useless,args = (E,))
    p2 = mp.Process(target = print_useless_2)

    p1.start()
    p2.start()

    _ = input()
    E.set()

    p1.join()
    print("waitng p2")
    p2.join()