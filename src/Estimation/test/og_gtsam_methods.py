import gtsam


if __name__ == '__main__':
    method_list =  dir(gtsam)

    for method in method_list:
        if 'factor' in method.lower():
            print method