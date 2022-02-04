import matplotlib.pyplot as plt
import matplotlib.patches as patches
import json

def plot_shape(ax, x_vert_list, y_vert_list, r_apriltag):

    # append
    x_vert_list_wrap = x_vert_list + [x_vert_list[0]]
    y_vert_list_wrap = y_vert_list + [y_vert_list[0]]

    # plot shape
    ax.plot(x_vert_list_wrap, y_vert_list_wrap)

    # plot apriltag
    rect = patches.Rectangle(xy=(r_apriltag[0] - 0.054/2, r_apriltag[1] - 0.054/2), 
        width=0.054, height=0.054)
    ax.add_patch(rect)
    return ax


if __name__ == "__main__":

    f = open("triangle.json")
    shape_data = json.load(f)


    fig, ax = plt.subplots(nrows=1, ncols=1)
    ax = plot_shape(ax=ax, 
        x_vert_list=shape_data["x_vert"], 
        y_vert_list=shape_data["y_vert"], 
        r_apriltag=shape_data['centroid_to_apriltag'])
    ax.set_aspect('equal', adjustable='box')
    
    plt.show()

    # print("hello")
