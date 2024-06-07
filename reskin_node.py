import hydra
from openteach.components import ReskinSensors

@hydra.main(version_base = '1.2', config_path = 'configs', config_name = 'reskin')
def main(configs):
    reskin = ReskinSensors(configs)
    processes = reskin.get_processes()

    for process in processes:
        process.start()

    for process in processes:
        process.join()

if __name__ == '__main__':
    main()