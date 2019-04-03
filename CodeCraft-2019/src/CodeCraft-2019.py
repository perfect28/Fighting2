# coding=utf-8
import logging
import sys
import time
from CarScheduler import CarScheduler

logging.basicConfig(level=logging.DEBUG,
                    filename='../logs/CodeCraft-2019.log',
                    format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S',
                    filemode='a')


def main():
    if len(sys.argv) != 6:
        logging.info('please input args: car_path, road_path, cross_path, answerPath')
        exit(1)

    car_path = sys.argv[1]
    road_path = sys.argv[2]
    cross_path = sys.argv[3]
    preset_answer_path = sys.argv[4]
    answer_path = sys.argv[5]

    logging.info("car_path is %s" % (car_path))
    logging.info("road_path is %s" % (road_path))
    logging.info("cross_path is %s" % (cross_path))
    logging.info("preset_answer_path is %s" % (preset_answer_path))
    logging.info("answer_path is %s" % (answer_path))

    # 正常执行
    start = time.clock()
    carScheduler = CarScheduler()
    carScheduler.work(car_path, road_path, cross_path, preset_answer_path)

    with open(answer_path, 'w') as f:
        for car in carScheduler.carList.values():
            for id, path_id in enumerate(car.path):
                if path_id > 100000:
                    car.path[id] = path_id - 100000
            f.write("(" + str(car.id) + ", " + str(car.start_time) + ", "
                    + ", ".join([str(a) for a in car.path]) + ")\n")
    end = time.clock()
    print('Final time: %s Seconds' % (end - start))


# to read input file
# process
# to write output file


if __name__ == "__main__":
    main()
