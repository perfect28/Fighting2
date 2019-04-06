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

    '''
    ../2-map-training-1/car.txt ../2-map-training-1/road.txt ../2-map-training-1/cross.txt  ../2-map-training-1/presetAnswer.txt ../2-map-training-1/answer.txt
    
        ../config-2/car.txt ../config-2/road.txt ../config-2/cross.txt  ../config-2/presetAnswer.txt ../config-2/answer.txt
        ../config-3/car.txt ../config-3/road.txt ../config-3/cross.txt  ../config-3/presetAnswer.txt ../config-3/answer.txt
    '''

    # # 测试最优参数
    # for weight in range(20, 41, 10):
    #     for carnum in range(1000, 4001, 1000):
    #         carScheduler = CarScheduler()
    #         carScheduler.weight = weight  # 车况权值，可调参数
    #         carScheduler.max_schedule_carnum = carnum  # 最大同时调度车辆数，可调参数
    #         carScheduler.work(car_path, road_path, cross_path, preset_answer_path)
    #         print("weight: %d, carnum: %d" % (weight, carnum))
    #         print("#" * 100)
    # return
    # 正常执行
    start = time.clock()
    carScheduler = CarScheduler()
    carScheduler.weight = 20
    carScheduler.max_schedule_carnum = 2000  # 最大同时调度车辆数，可调参数
    carScheduler.work(car_path, road_path, cross_path, preset_answer_path)

    with open(answer_path, 'w') as f:
        for car in carScheduler.carList.values():
            if not car.is_preset:
                for id, path_id in enumerate(car.path):
                    if path_id > 100000:
                        car.path[id] = path_id - 100000
                f.write("(" + str(car.id) + ", " + str(car.start_time) + ", "
                        + ", ".join([str(a) for a in car.path]) + ")\n")
    end = time.clock()
    print('\nFinal time: %s Seconds' % (end - start))

    scheduleTime = 0
    allScheduleTime = 0
    for car in carScheduler.carList.values():
        if car.is_priority == 1:
            scheduleTime = max(scheduleTime, car.end_time)
            allScheduleTime += car.end_time - car.start_time
    print("specialResult is Result {scheduleTime = %d, allScheduleTime = %d}"
          % (scheduleTime, allScheduleTime))

    scheduleTime = carScheduler.current_time
    allScheduleTime = 0
    for car in carScheduler.carList.values():
        allScheduleTime += car.end_time - car.start_time
        # if car.start_time > car.end_time:
        #     print(str(car.id) + " " + str(car.start_time) + " " + str(car.end_time))
    print("originResult is Result {scheduleTime = %d, allScheduleTime = %d}"
          % (scheduleTime, allScheduleTime))


# to read input file
# process
# to write output file

if __name__ == "__main__":
    main()
