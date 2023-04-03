import argparse
from enum import Enum

class data(Enum):
    PRFIX = 0
    CLASS = 1
    SKILL = 2
    PARAM = 3
    OBJCT = 4

parser = argparse.ArgumentParser(description='')
parser.add_argument('-file', type=str, help='', default='heron.turtle')
parser.add_argument('-mode', choices=['verify', 'weak', 'agressive'], help='Mode :)', default="verify")
parser.add_argument('-sort', choices=['true', 'false', 'True', 'False'], help='Sort', default='False')

def parse_prefix(line):
    line = line[8:-1]
    prefix_name, prefix_link = line.split(': ')

    _, prefix_link = prefix_link.split('<')
    prefix_link, _ = prefix_link.split('>')

    return prefix_name.strip(), prefix_link

def parse_object_type(line, file):
    _, types = line.split(' a ')
    types = [tp.strip() for tp in types.split(',')]

    if types[-1] == '':
        types.pop(-1)
        line = file.readline()
        while True:
            if line == '':
                raise RuntimeError('Expected "." or ";" but got EOF.')
            
            types.extend([tp.strip() for tp in line.split(',')])

            if types[-1][-1] == ';' or types[-1][-1] == '.':
                break

    end_of_object = types[-1][-1] == '.'
    types[-1] = types[-1][:-1].strip()

    return file, types, end_of_object

def parse_single_attribute(file):
    line = file.readline().strip()
    ind = line.find(' ')

    attribute_name = line[:ind]
    line = line[ind:].strip()

    attribute_values = []
    final_attribute = True
    first_line = True
    while True:
        if not first_line:
            line = file.readline().strip()
        else:
            first_line = False
        
        attribute_values.extend([att.strip() for att in line.split(',') if att.strip() != ''])

        if line[-1] == ';' or line[-1] == '.':
            break

    final_attribute = attribute_values[-1][-1] == '.'
    attribute_values[-1] = attribute_values[-1][:-1].strip()
    return file, (attribute_name, attribute_values), final_attribute

def parse_all_object_attributes(file):
    attributes = []
    final_attribute = False

    while not final_attribute:
        file, attribute, final_attribute = parse_single_attribute(file)
        attributes.append(attribute)
    
    return file, attributes

def parse_wm_object(line, file):
    object_name, _ = line.split(' a ')
    object_name = object_name.strip()

    file, object_types, end_of_object = parse_object_type(line, file)

    if not end_of_object:
        file, object_attributes = parse_all_object_attributes(file)
    else:
        object_attributes = []

    return file, object_name, object_types, object_attributes

def parse_turtle(file):
    wm_objects = []  # [(data enum, data), ...]
    # prefix = (name, link)
    # class = (name, attributes)
    # skill = (name, types, attributes)
    # param = (name, types, attributes)
    # object = (name, types, attributes)

    line = None
    new_object = False
    while line != '':
        line = file.readline()

        if '@prefix' in line:
            new_object = True
            wm_object = parse_prefix(line)
            wm_object_type = data.PRFIX
        elif line.strip() != '':
            new_object = True
            file, *wm_object = parse_wm_object(line, file)

            if 'owl:class' in line:
                wm_object_type = data.CLASS
            elif 'a skiros:Parameter' in line:
                wm_object_type = data.PARAM
            elif line.strip() != '':
                _, _, attributes = wm_object
                is_skill = False
                for _, attribute_values in attributes:
                    for vals in attribute_values:
                        if 'skiros:Parameter' in vals:
                            is_skill = True
                            break
                    if is_skill:
                        break

                if is_skill:
                    wm_object_type = data.SKILL
                else:
                    wm_object_type = data.OBJCT
        
        if new_object:
            new_object = False
            wm_objects.append((wm_object_type, wm_object))
    
    return wm_objects

def dismember_turtle(wm_objects):
    wm_prefix =  [(wm_object, i) for i, (object_type, wm_object) in enumerate(wm_objects) if object_type == data.PRFIX]
    wm_classes = [(wm_object, i) for i, (object_type, wm_object) in enumerate(wm_objects) if object_type == data.CLASS]
    wm_skills =  [(wm_object, i) for i, (object_type, wm_object) in enumerate(wm_objects) if object_type == data.SKILL]
    wm_params =  [(wm_object, i) for i, (object_type, wm_object) in enumerate(wm_objects) if object_type == data.PARAM]
    wm_objects = [(wm_object, i) for i, (object_type, wm_object) in enumerate(wm_objects) if object_type == data.OBJCT]

    return wm_prefix, wm_classes, wm_skills, wm_params, wm_objects

def verify_turtle(wm_prefix, wm_classes, wm_skills, wm_params, wm_objects):
    pass

def clean_turtle(turtle)
    pass

def reassemble_turtle(turtle):
    pass

def main():
    # Parse arguments
    args = parser.parse_args()

    with open(args.file) as file:
        turtle = parse_turtle(file)
    print('%s could be parsed.' % args.file)

    if args.mode == 'verify':
        verify_turtle(*dismember_turtle(turtle))
    else:
        raise NotImplementedError('Unsupported mode. Unreachable error :)')

if __name__ == '__main__':
    main()
