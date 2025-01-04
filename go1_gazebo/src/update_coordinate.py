#!/usr/bin/env python3

import yaml
import os
from ament_index_python.packages import get_package_share_directory

def update_robot_positions(input_file, output_file):
    # Чтение данных из входного файла
    with open(input_file, 'r') as infile:
        data = yaml.safe_load(infile)

    # Извлечение информации о роботах из vertices
    robots = []
    for vertex in data['levels']['L1']['vertices']:
        # Проверка длины списка и наличия атрибутов spawn_robot_name и spawn_robot_type
        if len(vertex) > 4 and 'spawn_robot_name' in vertex[4] and 'spawn_robot_type' in vertex[4]:
            # Извлечение x и y координат
            x = vertex[0]
            y = vertex[1]
            robot_name = vertex[3]

            # Формирование информации о роботе
            robot_info = {
                'name': robot_name,
                'x_pose': str(x/20.0078857),
                'y_pose': '-' + str(y/20.01035),
                'z_pose': '0.01',
            }
            robots.append(robot_info)

    # Чтение и обновление выходного файла
    with open(output_file, 'r') as outfile:
        output_data = yaml.safe_load(outfile)

    # Обновление данных роботов
    output_data['robots'] = robots

    # Настройка YAML форматирования для корректных отступов
    class IndentDumper(yaml.Dumper):
        def increase_indent(self, flow=False, indentless=False):
            return super(IndentDumper, self).increase_indent(flow, False)

    # Запись обновленных данных обратно в выходной файл с правильным форматированием
    with open(output_file, 'w') as outfile:
        yaml.dump(output_data, outfile, Dumper=IndentDumper, default_flow_style=False, sort_keys=False, indent=2)

def main():
    input_file_path = os.path.join(get_package_share_directory('ff_examples_ros2'), 'maps','cambrdge_student_house', 'cambridge.building.yaml')
    output_file_path = os.path.join(get_package_share_directory('turtlebot3_multi_robot'), 'params', 'robots.yaml')

    update_robot_positions(input_file_path, output_file_path)

if __name__ == '__main__':
    main()
