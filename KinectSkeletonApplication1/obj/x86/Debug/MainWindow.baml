   M S B A M L   `   `   `  ����^  ZMicrosoft.Samples.Kinect.WpfViewers, Version=0.0.0.0, Culture=neutral, PublicKeyToken=null�^clr-namespace:Microsoft.Samples.Kinect.WpfViewers;assembly=Microsoft.Samples.Kinect.WpfViewers#Microsoft.Samples.Kinect.WpfViewers  5   	    KinectSkeletonApplication1+   %KinectSkeletonApplication1.MainWindow   5   	   \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   u
WpfViewers^clr-namespace:Microsoft.Samples.Kinect.WpfViewers;assembly=Microsoft.Samples.Kinect.WpfViewers     5   	     � Title$  
MainWindow��5   	   $	��800��6   $
��1600��6)   .��5      �  � ColumnDefinitions 5   
   �� 5       � RowDefinitions 5
   
   �� 5      .+�5   
   �� -   5         ��Name$ 
videoImage��5      $��43,69,749,100q�6$   $	��800��6;   $	��600��6G   �� 5   
   
������ $��0,69,792,100q�6*   $��Right=�6@   $	��800��6\   $	��600��6h   .+�5      >   8Microsoft.Samples.Kinect.WpfViewers.KinectSkeletonViewer  -   5          Name$ kinectSkeletonViewer1��5   .   $	��600��6M   $	��800��6Z    �� Left$ 39��6�   5   /Microsoft.Samples.Kinect.WpfViewers.ImageViewer   Kinect 6f   �� *Kinect+ �� ElementName$ kinectSensorChooser1��6�   A� 5      -   5         A�Name$ ellipseLeftHand��5   8   
���  ��$��20��6!   $��20��6-   
�������$	 309��6`   
	 �� Top$		 305��6r   A� 5      -   5        $ ellipseRightHand��5   8   
���  ��$��20��6!   $��20��6-   
�������$	 552��6a   $		 310��6s   A� 5      -   5        $ ellipsehead��5   8   
���  ��$��20��6!   $��20��6-   
�������$	 412��6\   $		 166��6n   A� 5      -   5        $ ellipseLeftFoot��5      
���  ��$��20��6:   $	 387��6F   
�������$		 479��6g   $��20��6x   A� 5      -   5        $ ellipseRightFoot��5      
���  ��$��20��6;   $	 502��6G   
�������$		 479��6h   $��20��6y   �� 5      $��
L__Hand__X��6   $	 857��6)   $		 310��6;   $��65��6L   $��25��6W   A� 5      -   5        $ ellipseRightShoulder��5   8   
���  ��$��20��6!   $��20��6-   
�������$	 478��6e   $		 264��6w   A� 5      -	   5        $ ellipseLeftShoulder��5   8   
���  ��$��20��6!   $��20��6-   
�������$	 387��6d   $		 264��6v   5      �� 5   
   -
   5        
 ��Name$
 up��5      $��往上��6   $
��Left=�6*   $��866,141,0,0q�6E   $	��Top=�6Z   $��75��6r   $��19��6�   �� 5   
   -   5        $
 bzero��5      $��歸零��6    $
��Left=�6-   $��976,141,0,0q�6H   $	��Top=�6]   $��75��6u   $��19��6�   �� 5   
   -   5        $

 down��5      $��往下��6   $
��Left=�6,   $��1086,141,0,0q�6G   $	��Top=�6]   $��75��6u   $��19��6�   �� 5   
   -   5         ��Name$ Gesture_test��5      $
��Left=�6(   $��23��6C   $��1007,194,0,0q�6O    �� TextWrapping$
 Wrap=�6e   $	��Top=�6y   $	��120��6�   �� 5   
   $��手勢動作��6   $
��Left=�6   $��934,192,0,0q�6:   $	��Top=�6O   $��25��6g   $��58��6s   �� 5   
   -   5        $ AngleTxt��5      $
 Wrap=�6$   $��目前角度:��68   $��953,69,519,677q�6E   �� 5   
   -   5        $ 
L__Hand__X��5      $
��Left=�6&   $��23��6A   $��934,384,0,0q�6M   $
 Wrap=�6b   $	��Top=�6v   $	��120��6�   �� 5    
   -   5        $ 
L__Hand__Y��5       $
��Left=�6&   $��23��6A   $��934,423,0,0q�6M   $
 Wrap=�6b   $	��Top=�6v   $	��120��6�   �� 5!   
   $��
L__Hand__Y��6   $
��Left=�6%   $��859,423,0,0q�6@   $	��Top=�6U   $��65��6m   $��25��6x   �� 5"   
   -   5        $ 
R__Hand__X��5"      $
��Left=�6&   $��23��6A   $��1201,383,0,0q�6M   $
 Wrap=�6c   $	��Top=�6w   $	��120��6�   �� 5#   
   -   5        $ 
R__Hand__Y��5#      $
��Left=�6&   $��23��6A   $��1201,423,0,0q�6M   $
 Wrap=�6c   $	��Top=�6w   $	��120��6�   �� 5$   
   $��
R__Hand__X��6   $
��Left=�6%   $��1114,384,0,0q�6@   $	��Top=�6V   $��70��6n   $��25��6y   �� 5%   
   $��
R__Hand__Y��6   $
��Left=�6%   $��1114,421,0,0q�6@   $	��Top=�6V   $��70��6n   $��25��6y   �� 5&   
   -   5        $ 
L__Foot__X��5&      $
��Left=�6&   $��23��6A   $��934,533,0,0q�6M   $
 Wrap=�6b   $	��Top=�6v   $	��120��6�   �� 5'   
   -   5        $ 
L__Foot__Y��5'      $
��Left=�6&   $��23��6A   $��934,583,0,0q�6M   $
 Wrap=�6b   $	��Top=�6v   $	��120��6�   �� 5(   
   $��
L__Foot__X��6   $
��Left=�6%   $��859,533,0,0q�6@   $	��Top=�6U   �� 5)   
   $��
L__Foot__Y��6   $
��Left=�6%   $��859,581,0,0q�6@   $	��Top=�6U   �� 5*   
   -   5        $ 
R__Foot__X��5*      $
��Left=�6&   $��23��6A   $��1201,526,0,0q�6M   $
 Wrap=�6c   $	��Top=�6w   $	��120��6�   �� 5+   
   -   5        $ 
R__Foot__Y��5+      $
��Left=�6&   $��23��6A   $��1201,576,0,0q�6M   $
 Wrap=�6c   $	��Top=�6w   $	��120��6�   �� 5,   
   $��
R__Foot__X��6   $
��Left=�6%   $��1114,526,0,0q�6@   $	��Top=�6V   �� 5-   
   $��
R__Foot__Y��6   $
��Left=�6%   $��1114,574,0,0q�6@   $	��Top=�6V   �� 5.   
   -   5        $ Head__X��5.      $
��Left=�6#   $��23��6>   $��1097,293,0,0q�6J   $
 Wrap=�6`   $	��Top=�6t   $	��120��6�   �� 5/   
   $��Head__X��6   $
��Left=�6"   $��1033,291,0,0q�6=   $	��Top=�6S   �� 50   
   -   5        $ Head__Y��50      $
��Left=�6#   $��23��6>   $��1097,334,0,0q�6J   $
 Wrap=�6`   $	��Top=�6t   $	��120��6�   �� 51   
   $��Head__Y��6   $
��Left=�6"   $��1033,334,0,0q�6=   $	��Top=�6S   8   2Microsoft.Samples.Kinect.WpfViewers.KinectSkeleton  52   
   $
��Left=�6$   $	��100��6?   $��
841,23,0,0q�6L   $	��Top=�6`   $	��100��6x   �� 53   
   -   5        $ L__Shoulder__X��53      $
��Left=�6*   $��23��6E   $��934,459,0,0q�6Q   $
 Wrap=�6f   $	��Top=�6z   $	��120��6�   �� 54   
   -   5        $ R__Shoulder__X��54      $
��Left=�6*   $��23��6E   $��1201,461,0,0q�6Q   $
 Wrap=�6g   $	��Top=�6{   $	��120��6�   �� 55   
   $��L__Shoulder__X��6   $
��Left=�6)   $��843,457,0,0q�6D   $	��Top=�6Y   �� 56   
   $��R__Shoulder__X��6   $
��Left=�6)   $��1097,459,0,0q�6D   $	��Top=�6Z   �� 57   
   -   5        $ L__Shoulder__Y��57      $
��Left=�6*   $��23��6E   $��934,493,0,0q�6Q   $
 Wrap=�6f   $	��Top=�6z   $	��120��6�   �� 58   
   $��L__Shoulder__Y��6   $
��Left=�6)   $��848,491,0,0q�6D   $	��Top=�6Y   �� 59   
   $��R__Shoulder__Y��6   $
��Left=�6)   $��1097,493,0,0q�6D   $	��Top=�6Z   �� 5:   
   -   5        $ R__Shoulder__Y��5:      $
��Left=�6*   $��23��6E   $��1201,495,0,0q�6Q   $
 Wrap=�6g   $	��Top=�6{   $	��120��6�   �� 5;   
   $��右手舉起角度��6   $
��Left=�6!   $��1358,379,0,0q�6<   $	��Top=�6R   �� 5<   
   -   5        $ R_Hand_Angle��5<      $
��Left=�6(   $��23��6C   $��1462,384,0,0q�6O   $
 Wrap=�6e   $	��Top=�6y   $	��120��6�   �� 5=   
   $��左手舉起角度��6   $
��Left=�6!   $��1358,423,0,0q�6<   $	��Top=�6R   �� 5>   
   -   5        $ L_Hand_Angle��5>      $
��Left=�6(   $��23��6C   $��1462,421,0,0q�6O   $
 Wrap=�6e   $	��Top=�6y   $	��120��6�   �� 5?   
   -   5        $
 	Zigbee_go��5?      $��確認��6$   $
��Left=�61   $��1447,195,0,0q�6L   $	��Top=�6b   $��75��6z   6�   �� 5@   
   -    5         ��Name$ Zigbee��5@      $
��Left=�6#   $��1289,192,0,0q�6>   $	��Top=�6T   $	��120��6l   =   7Microsoft.Samples.Kinect.WpfViewers.KinectSensorChooser  5A   
   -!   5          Name$ kinectSensorChooser1��5A   )   $
��Left=�6G   $��
1254,0,0,0q�6b   $	��Top=�6v   5C      5D      