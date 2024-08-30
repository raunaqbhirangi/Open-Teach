import hydra
from openteach.components import DigitSensors

@hydra.main(version_base = '1.2', config_path = 'configs', config_name = 'digit')
def main(configs):
    digit = DigitSensors(configs)
    processes = digit.get_processes()

    for process in processes:
        process.start()

    for process in processes:
        process.join()

if __name__ == '__main__':
    main()