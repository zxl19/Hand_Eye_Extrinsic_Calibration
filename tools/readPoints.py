import configparser


# *Read config file
config = configparser.ConfigParser()
config.read('config.ini')
input_point_path = config['DEFAULT']['input_point_path']
output_point_path = config['DEFAULT']['output_point_path']


output_file = open(output_point_path, 'w')
with open(input_point_path, 'r') as input_file:
    for line in input_file:
        if line[0] == '#':
            continue
        else:
            data = line.split(' ')
            X = data[1]
            Y = data[2]
            Z = data[3]
            R = data[4]
            G = data[5]
            B = data[6]
            points = [X, Y, Z, R, G, B]
            output_file.write(' '.join(points))
            output_file.write('\n')