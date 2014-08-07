using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Text;
using System.Data;
using System.Drawing;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;
using System.Linq;
using System.IO.Ports;


namespace KinectSkeletonApplication1
{
    public partial class MainWindow : Window
    {
        //Instantiate the Kinect runtime. Required to initialize the device.
        //IMPORTANT NOTE: You can pass the device ID here, in case more than one Kinect device is connected.
        KinectSensor sensor = KinectSensor.KinectSensors[0];//宣告 KinectSensor
        //變數初始化定義
        byte[] pixelData;
        Skeleton[] skeletons;

        #region ZigbeePortc宣告
        internal System.IO.Ports.SerialPort serialport = new System.IO.Ports.SerialPort();//宣告連結埠
        internal bool serialportopen = false;           
        #endregion ZigbeePort選擇

        //Left
        bool isLeftHandOverHead = false;
        bool isLeftHelloGestureActive = false;
        //Right
        bool isRightHelloGestureActive = false;
        bool isRightHandOverHead = false;
        bool CheckGesture_ready = false;
        
               

        public MainWindow()
        {
            InitializeComponent();
            
            //Runtime initialization is handled when the window is opened. When the window
            //is closed, the runtime MUST be unitialized.
            this.Loaded += new RoutedEventHandler(MainWindow_Loaded);
            this.Unloaded += new RoutedEventHandler(MainWindow_Unloaded);
            
            #region Zigbee顯示全部com          
            Zigbee.Items.Clear();
            foreach (string com in System.IO.Ports.SerialPort.GetPortNames())//取得所有可用的連接埠
            {
                Zigbee.Items.Add(com);
            }
            #endregion Zigbee顯示全部com
            sensor.ColorStream.Enable();//開啟彩色影像      
     
            //攝影機控制角度
            up.Click += up_Click;
            bzero.Click +=bzero_Click;
            down.Click += down_Click;

            #region 平滑處理，防止高頻率微小抖動和突發大跳動造成的關節雜訊
            var parameters = new TransformSmoothParameters
            {
                Smoothing = 0.3f,
                Correction = 0.0f,
                Prediction = 0.0f,
                JitterRadius = 1.0f,
                MaxDeviationRadius = 0.5f
            };
            #endregion 平滑處理，防止高頻率微小抖動和突發大跳動造成的關節雜訊

            sensor.SkeletonStream.Enable(parameters);//載入平滑處理參數
            sensor.SkeletonStream.Enable();//開啟骨架追蹤

           
        }



        #region ZigbeePort設定
        
        private void Zigbee_go_Click(object sender, RoutedEventArgs e)
        {
            if (serialportopen == false && !serialport.IsOpen)
            {
                try
                {
                    //設定連接埠為9600、n、8、1、n
                    serialport.PortName = Zigbee.Text;
                    serialport.BaudRate = 115200;
                    serialport.DataBits = 8;
                    serialport.Parity = System.IO.Ports.Parity.None;
                    serialport.StopBits = System.IO.Ports.StopBits.One;
                    serialport.Encoding = Encoding.Default;//傳輸編碼方式
                    serialport.Open();
                    serialportopen = true;
                    
                    Zigbee_go.Content = "中斷";
                    serialport.Write("1"); 
                   
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.ToString());
                }
            }
            else if (serialportopen == true && serialport.IsOpen)
            {
                try
                {
                    serialport.Close();
                    if (!serialport.IsOpen)
                    {
                        serialportopen = false;                        
                        Zigbee_go.Content = "連線";    
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.ToString());
                }
            }
        }
         
        #endregion ZigbeePort設定
               
        #region 骨架追蹤，處理部分
        //骨架追蹤，處理函數
        void runtime_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            bool receivedData = false;

            using (SkeletonFrame SFrame = e.OpenSkeletonFrame())
            {
                if (SFrame == null)
                {
                    // The image processing took too long. More than 2 frames behind.
                }
                else
                {
                    skeletons = new Skeleton[SFrame.SkeletonArrayLength];
                    SFrame.CopySkeletonDataTo(skeletons);
                    receivedData = true;
                }
            }
            //定義關節
            if (receivedData)
            {

                Skeleton currentSkeleton = (from s in skeletons
                                            where s.TrackingState == SkeletonTrackingState.Tracked
                                            select s).FirstOrDefault();

                if (currentSkeleton != null)
                {
                    //取得骨架關節點 3D(X、Y、Z)座標值。
                   /* SetEllipsePosition(head, currentSkeleton.Joints[JointType.Head]);
                    SetEllipsePosition(leftHand, currentSkeleton.Joints[JointType.HandLeft]);
                    SetEllipsePosition(rightHand, currentSkeleton.Joints[JointType.HandRight]);
                    SetEllipsePosition(Rightfoot, currentSkeleton.Joints[JointType.FootRight]);*/
                    var head = currentSkeleton.Joints[JointType.Head];//頭
                    var rightHand = currentSkeleton.Joints[JointType.HandRight];//手掌
                    var leftHand = currentSkeleton.Joints[JointType.HandLeft];
                    var leftFoot = currentSkeleton.Joints[JointType.FootLeft];//腳掌
                    var rightFoot = currentSkeleton.Joints[JointType.FootRight];
                    var rightShoulder = currentSkeleton.Joints[JointType.ShoulderRight];//肩膀
                    var leftShoulder = currentSkeleton.Joints[JointType.ShoulderLeft];

                    //紅球連動
                    SetEllipsePosition(ellipsehead, head, false);
                    SetEllipsePosition(ellipseLeftHand, leftHand, isLeftHelloGestureActive);
                    SetEllipsePosition(ellipseRightHand, rightHand, isRightHelloGestureActive);
                    SetEllipsePosition(ellipseRightFoot, rightFoot, isRightHelloGestureActive);
                    SetEllipsePosition(ellipseLeftFoot, leftFoot, isLeftHelloGestureActive);
                    SetEllipsePosition(ellipseLeftShoulder, leftShoulder, isLeftHelloGestureActive);
                    SetEllipsePosition(ellipseRightShoulder, rightShoulder, isLeftHelloGestureActive);

                    ProcessGesture(head, rightHand, leftHand, rightFoot, leftFoot, leftShoulder, rightShoulder);//呼叫處理手勢函數
                }
            }
        }
        #endregion 骨架追蹤，處理部分

        #region 判斷手勢部分&顯示數值&角度計算
        //判斷處理手勢函數
        private void ProcessGesture(Joint head, Joint rightHand, Joint leftHand, Joint rightFoot, Joint leftFoot, Joint leftShoulder, Joint rightShoulder)
        {
            #region 讀取各點座標
            Head__X.Text = "" + head.Position.X;//讀取頭YX座標
            Head__Y.Text = "" + head.Position.Y;
            L__Shoulder__X.Text = "" + leftShoulder.Position.X;//讀取左肩膀XY座標
            L__Shoulder__Y.Text = "" + leftShoulder.Position.Y;
            R__Shoulder__X.Text = "" + rightShoulder.Position.X;//讀取右肩膀XY座標
            R__Shoulder__Y.Text = "" + rightShoulder.Position.Y;
            R__Hand__X.Text = "" + rightHand.Position.X;//讀取右手掌XY座標
            R__Hand__Y.Text = "" + rightHand.Position.Y;
            L__Hand__X.Text = "" + leftHand.Position.X; //讀取左手掌XY座標
            L__Hand__Y.Text = "" + leftHand.Position.Y;
            #endregion 讀取各點座標

            //==="右手"揮動判斷式===
            if (rightHand.Position.X > head.Position.X + 3.5)
            {
                CheckGesture();
                if (CheckGesture_ready)
                {
                    isRightHelloGestureActive = true;
                    Gesture_test.Text = "右手揮動";
                   // MessageBox.Show("右手揮動 000 ");
                    //System.Windows.Forms.SendKeys.SendWait("{Right}");
                   
                   
                }
            }
            else
            {
                isRightHelloGestureActive = false;    
                
               
            }

            //==="左手"揮動判斷式===
            if (leftHand.Position.X < head.Position.X - 0.5)
            {
                CheckGesture();
                if (CheckGesture_ready)
                {
                    isLeftHelloGestureActive = true;
                    Gesture_test.Text = "左手揮動";
                   // MessageBox.Show("000 左手揮動");
                    //System.Windows.Forms.SendKeys.SendWait("{Left}");
                    
                }
            }
            else
            {
                isLeftHelloGestureActive = false;
                
                
            }

            //==="右手"舉高判斷式===
            if (rightHand.Position.Y > head.Position.Y+3.5)
            {
                CheckGesture();
                if (CheckGesture_ready)
                {
                    isRightHandOverHead = true;
                    Gesture_test.Text = "右手舉高: " ;
                    //MessageBox.Show("右手舉高 000 000 000 ");
                    
                    

                }
            }
            else
            {
                isRightHandOverHead = false;

                //公式:角度=ACOS(肩膀X-手掌X/開根號((肩膀X-手掌X)^2+(肩膀Y-手掌Y)^2))
                double SxHx = rightHand.Position.X - rightShoulder.Position.X;//肩膀X-手掌X
                double SyHy = Math.Pow(rightHand.Position.Y - rightShoulder.Position.Y, 2);//肩膀Y-手掌Y)^2
                double SqrSH = Math.Sqrt(Math.Pow(SxHx, 2) + SyHy);//開根號((肩膀X-手掌X)^2+(肩膀Y-手掌Y)^2)
                double AcosSH = Math.Acos(SxHx / SqrSH);//計算結果
                double DegAcosSH = 90+(AcosSH / 3.14) * 180;//輸出轉換角度=(Acos/3.14)*180
                double DegAcosSH2 = 90 - (AcosSH / 3.14) * 180;//輸出轉換角度=(Acos/3.14)*180
                if (rightHand.Position. Y > leftShoulder.Position.Y)
                {
                     R_Hand_Angle.Text = "" + DegAcosSH;
                    //達到設定則傳送1
                    serialport.Write("1"); 
                }
                else
                {
                    DegAcosSH2 = DegAcosSH2;
                    R_Hand_Angle.Text = "" + DegAcosSH2;
                    //無達到設定則傳送空白
                   /* serialport.Write("0");
                    serialport.DiscardInBuffer();*/
                }

            }

            //==="左手"舉高判斷式===
            if (leftHand.Position.Y > head.Position.Y)
            {
                CheckGesture();
                if (CheckGesture_ready)
                {
                    isLeftHandOverHead = true;
                    Gesture_test.Text = "左手舉高";
                    //MessageBox.Show("000 000 000 左手舉高");
                }
            }
            else
            {
                isLeftHandOverHead = false;

            }    
        }
        #endregion 判斷手勢部分&顯示數值&角度計算

        private void CheckGesture()
        {
            if (!isLeftHelloGestureActive && !isRightHelloGestureActive && !isRightHandOverHead && !isLeftHandOverHead)
            {
                CheckGesture_ready = true;
            }
            else
            {
                CheckGesture_ready = false;
            }
        }


        //This method is used to position the ellipses on the canvas
        //according to correct movements of the tracked joints.

        //IMPORTANT NOTE: Code for vector scaling was imported from the Coding4Fun Kinect Toolkit
        //available here: http://c4fkinect.codeplex.com/
        //I only used this part to avoid adding an extra reference.

        #region 設定影像顯示位置&2D計算
        //設定圖案位置
        private void SetEllipsePosition(Ellipse ellipse, Joint joint, bool isHighlighted)
        {
            //將3D 座標轉換成螢幕上大小，如640*320 的 2D 座標值
            Microsoft.Kinect.SkeletonPoint vector = new Microsoft.Kinect.SkeletonPoint();
            vector.X = ScaleVector(800, joint.Position.X);
            vector.Y = ScaleVector(600, -joint.Position.Y);
            vector.Z = joint.Position.Z; // Z值原封不動

            Joint updatedJoint = new Joint();
            updatedJoint = joint;
            updatedJoint.TrackingState = JointTrackingState.Tracked;
            updatedJoint.Position = vector;
            

            //得到 2D座標值(X、Y)後，將值設定為圖案顯示的位置
            Canvas.SetLeft(ellipse, updatedJoint.Position.X+80);
            Canvas.SetTop(ellipse, updatedJoint.Position.Y-0);
        }

        //處理螢幕大小2D座標值
        private float ScaleVector(int length, float position)
        {
            float value = (((((float)length) / 1f) / 2f) * position) + (length / 2);
            if (value > length)
            {
                return (float)length;
            }
            if (value < 0f)
            {
                return 0f;
            }
            return value;
        }
        #endregion 設定影像顯示位置&2D計算

        #region MainWindow 卸載
        void MainWindow_Unloaded(object sender, RoutedEventArgs e)
        {
            sensor.Stop();
        }
        #endregion MainWindow 卸載
        
        #region MainWindow 載入
        void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            kinectSensorChooser1.KinectSensorChanged += new DependencyPropertyChangedEventHandler(kinectSensorChooser1_KinectSensorChanged);

            sensor.SkeletonFrameReady += runtime_SkeletonFrameReady;
            sensor.ColorFrameReady += runtime_VideoFrameReady;
            sensor.Start();
        }
        #endregion MainWindow 載入
        
        #region Chooser讀取骨架
        //使用Chooser讀出骨架
        private void kinectSensorChooser1_KinectSensorChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            KinectSensor old = (KinectSensor)e.OldValue;

            StopKinect(old);

            KinectSensor sensor = (KinectSensor)e.NewValue;

            if (sensor == null)
            {
                return;
            }

            var parameters = new TransformSmoothParameters
            {
                Smoothing = 0.3f,
                Correction = 0.0f,
                Prediction = 0.0f,
                JitterRadius = 1.0f,
                MaxDeviationRadius = 0.5f
            };
            sensor.SkeletonStream.Enable(parameters);

            sensor.SkeletonStream.Enable();

            sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);

            try
            {
                sensor.Start();
            }
            catch (System.IO.IOException)
            {
                kinectSensorChooser1.AppConflictOccurred();
            }
        }
        #endregion Chooser讀取骨架

        #region 處理掉舊的資訊
        private void StopKinect(KinectSensor old)
        {
            if (sensor != null)
            {
                if (sensor.IsRunning)
                {
                    //stop sensor 
                    sensor.Stop();

                    //stop audio if not null
                    if (sensor.AudioSource != null)
                    {
                        sensor.AudioSource.Stop();
                    }
                }

            }
        }
        #endregion 處理掉舊的資訊
        
        #region 處理彩色影像
        //彩色影像，處理函數
        void runtime_VideoFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            bool receivedData = false;

            using (ColorImageFrame CFrame = e.OpenColorImageFrame())
            {
                if (CFrame == null)
                {
                    // The image processing took too long. More than 2 frames behind.
                }
                else
                {
                    pixelData = new byte[CFrame.PixelDataLength];
                    CFrame.CopyPixelDataTo(pixelData);
                    receivedData = true;
                }
            }

            if (receivedData)
            {
                BitmapSource source = BitmapSource.Create(640, 480, 96, 96,
                        PixelFormats.Bgr32, null, pixelData, 640 * 4);

                videoImage.Source = source;
            }
        }
        #endregion 處理彩色影像

        #region 攝影機控制上下
        //攝影機按鈕控制區
        KinectSensor motor = KinectSensor.KinectSensors[0];
        //往上
        private void up_Click(object sender, RoutedEventArgs e)
        {
            if (motor.ElevationAngle + 5 > motor.MaxElevationAngle)
            {
                motor.ElevationAngle = motor.MaxElevationAngle;
            }
            else
            {
                motor.ElevationAngle += 5;
            }
            AngleTxt.Text = "目前角度: " + motor.ElevationAngle;
            System.Threading.Thread.Sleep(1000);
        }
        //歸零
        private void bzero_Click(object sender, RoutedEventArgs e)
        {
            motor.ElevationAngle = 0;
            AngleTxt.Text = "目前角度: " + motor.ElevationAngle;
        }
        //往下
        private void down_Click(object sender, RoutedEventArgs e)
        {
            if (motor.ElevationAngle -5 < motor.MinElevationAngle)
            {
                motor.ElevationAngle = motor.MinElevationAngle;
            }
            else
            {
                motor.ElevationAngle -= 5;
            }
            AngleTxt.Text = "目前角度: " + motor.ElevationAngle;
            System.Threading.Thread.Sleep(1000);
        }
        #endregion 攝影機控制上下



        

    }


}