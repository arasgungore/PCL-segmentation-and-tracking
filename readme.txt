Please make sure that necessary libraries are installed to your computer. (OpenCV, pcl).

The program has three files such as main.cpp, source.cpp and lib.h. source.cpp has functions in it.

dataset folder must be in the same folder with main.cpp, wrt path in the program. 
The program uses a part of the scannet dataset. From frame-005000.color.jpg to frame-005020.color.jpg

The code works on two frames. The current configuration is working with the first and the last images of the dataset
to show object tracking well.
When the user run the main.cpp after a while, the viewer of the pcl library opens. The first viewer shows the point cloud data
of the first frame, after pressing 'Esc', program starts to do segmentation then shows the second viewer as segmented version of
the image. The user can skip all viewers with pressing 'Esc' button.
In the third viewer, the program shows flat surfaces on the original cloud. 

On the second tour of the viewers, in the last viewer, the user can see the object tracking and a list of N largest segments, current visibility (yes/no), number of occurrences on the terminal.

The user can change frames via processFrames() function and change path also from basePath variable. 

You can find example images in the dataset file. Normal values and therefore flat surfaces sensitive so sometimes you can get bad results for different images.

Because of the big size of the dataset, we only added 3 frame examples in out project file. 

For any problem, please contact us:
arif.yilmaz@boun.edu.tr - Arif Yılmaz 2021701138
arasgungore09@gmail.com - Aras Güngöre 2018401117
