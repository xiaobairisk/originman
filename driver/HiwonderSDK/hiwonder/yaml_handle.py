import yaml
lab_file_path = '/userdata/dev_ws/src/originman/driver/color_space.yaml'
servo_file_path = '/userdata/dev_ws/src/originman/driver/servo_config.yaml'

def get_yaml_data(yaml_file):
    file = open(yaml_file, 'r', encoding='utf-8')
    file_data = file.read()
    file.close()
    
    data = yaml.load(file_data, Loader=yaml.FullLoader)
    
    return data

def save_yaml_data(data, yaml_file):
    file = open(yaml_file, 'w', encoding='utf-8')
    yaml.dump(data, file)
    file.close()