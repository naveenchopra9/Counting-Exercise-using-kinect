 //------------------------------------------------------------------------------
// <copyright file="KinectSkeletonViewer.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.WpfViewers
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel; 
    using System.Windows;
    using System.Windows.Controls;
    using System.Windows.Media;
    using System.Windows.Shapes;
    using Microsoft.Kinect;
    using System.IO;
    using System.Linq;
    using System.Text;
    using System.Diagnostics;
    using System.Windows.Media.Media3D;
    public enum ImageType
    {
        Color,
        Depth,
    }

    internal enum TrackingMode
    {
        DefaultSystemTracking,
        Closest1Player,
        Closest2Player,
        Sticky1Player,
        Sticky2Player,
        MostActive1Player,
        MostActive2Player
    }

    /// <summary>
    /// Interaction logic for KinectSkeletonViewer.xaml
    /// </summary>
    public partial class KinectSkeletonViewer : ImageViewer, INotifyPropertyChanged
    {
        private const float ActivityFalloff = 0.98f;
        private readonly List<ActivityWatcher> recentActivity = new List<ActivityWatcher>();
        private readonly List<int> activeList = new List<int>();
        private List<KinectSkeleton> skeletonCanvases;
        private List<Dictionary<JointType, JointMapping>> jointMappings = new List<Dictionary<JointType, JointMapping>>();
        private Skeleton[] skeletonData;
        static float result = 0;
        static int countnumber1= 0,countnumber2=0,countnumber3=0,countnumber4=0,tempcount1=0,tempcount2=0,tempcount3=0,tempcount4=0;
        static double LeftElbowAngle=0.0f;
        static double rotationAngle=0.0f;
        static double rottwohand=0.0f;
        static double footrot=0.0f;
        public KinectSkeletonViewer()
        {
            InitializeComponent();
            this.ShowJoints = true;
            this.ShowBones = true;
            this.ShowCenter = true;
        }
        
        public bool ShowBones { get; set; }

        public bool ShowJoints { get; set; }

        public bool ShowCenter { get; set; }

        public ImageType ImageType { get; set; }

        internal TrackingMode TrackingMode { get; set; }

        public void HideAllSkeletons()
        {
            if (this.skeletonCanvases != null)
            {
                foreach (KinectSkeleton skeletonCanvas in this.skeletonCanvases)
                {
                    skeletonCanvas.Reset();
                }
            }
        }

        protected override void OnKinectChanged(KinectSensor oldKinectSensor, KinectSensor newKinectSensor)
        {
            if (oldKinectSensor != null)
            {
                oldKinectSensor.AllFramesReady -= this.KinectAllFramesReady;
                this.HideAllSkeletons();
            }

            if (newKinectSensor != null && newKinectSensor.Status == KinectStatus.Connected)
            {
                newKinectSensor.AllFramesReady += this.KinectAllFramesReady;
            }
        }

        private void KinectAllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            // Have we already been "shut down" by the user of this viewer, 
            // or has the SkeletonStream been disabled since this event was posted?
            if ((this.Kinect == null) || !((KinectSensor)sender).SkeletonStream.IsEnabled)
            {
                return;
            }
            // File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", "  0  ");
            bool haveSkeletonData = false;

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
             //   File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", "  1  ");
                if (skeletonFrame != null)
                {
                    if (this.skeletonCanvases == null)
                    {
                       // File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", "  2  ");
                        this.CreateListOfSkeletonCanvases();
                    }

                    if ((this.skeletonData == null) || (this.skeletonData.Length != skeletonFrame.SkeletonArrayLength))
                    {
                        this.skeletonData = new Skeleton[skeletonFrame.SkeletonArrayLength];
                      //  File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", "  3  ");
                    }

                    skeletonFrame.CopySkeletonDataTo(this.skeletonData);

                    haveSkeletonData = true;
                }
            }

            if (haveSkeletonData)
            {  
                using (DepthImageFrame depthImageFrame = e.OpenDepthImageFrame())
                {
                    if (depthImageFrame != null)
                    {
                     
                       // File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", "  4  ");
                        int trackedSkeletons = 0;
                        foreach (Skeleton skeleton in this.skeletonData)
                        {
                   
                            if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                            {
                             //   File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt","Tracked"); 
                            }
                            else if (skeleton.TrackingState == SkeletonTrackingState.PositionOnly)
                            {
                               // File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", trackedSkeletons.ToString() + "  10  ");
                           
                            }
                            else if (skeleton.TrackingState == SkeletonTrackingState.NotTracked)
                            {
                               //File.AppendAllText(@"C:\Users\pk1601cs33\hell.txt", "Nottrack");
                          
                            }
                            Dictionary<JointType, JointMapping> jointMapping = this.jointMappings[trackedSkeletons];
                            jointMapping.Clear();

                            KinectSkeleton skeletonCanvas = this.skeletonCanvases[trackedSkeletons++];
                            skeletonCanvas.ShowBones = this.ShowBones;
                            skeletonCanvas.ShowJoints = this.ShowJoints;
                            skeletonCanvas.ShowCenter = this.ShowCenter;
   
                            foreach (Joint joint in skeleton.Joints)
                            {
                                Point mappedPoint = this.GetPosition2DLocation(depthImageFrame, joint.Position);
                                jointMapping[joint.JointType] = new JointMapping
                                    {
                                        Joint = joint, 
                                        MappedPoint = mappedPoint
                                    };

                                Joint asd = joint;
                                if (asd.TrackingState == JointTrackingState.Tracked)
                                {
                                

                                    Vector3D elbowl = new Vector3D(skeleton.Joints[JointType.ElbowLeft].Position.X, skeleton.Joints[JointType.ElbowLeft].Position.Y, skeleton.Joints[JointType.ElbowLeft].Position.Z);
                                    Vector3D WristLeft = new Vector3D(skeleton.Joints[JointType.WristLeft].Position.X, skeleton.Joints[JointType.WristLeft].Position.Y, skeleton.Joints[JointType.WristLeft].Position.Z);
                                    Vector3D ShoulderLeft = new Vector3D(skeleton.Joints[JointType.ShoulderLeft].Position.X, skeleton.Joints[JointType.ShoulderLeft].Position.Y, skeleton.Joints[JointType.ShoulderLeft].Position.Z);

                                    Vector3D Head = new Vector3D(skeleton.Joints[JointType.ShoulderCenter].Position.X, skeleton.Joints[JointType.ShoulderCenter].Position.Y, skeleton.Joints[JointType.ShoulderCenter].Position.Z);
                                    Vector3D Neck = new Vector3D(skeleton.Joints[JointType.Spine].Position.X, skeleton.Joints[JointType.Spine].Position.Y, skeleton.Joints[JointType.Spine].Position.Z);
                                   
                                    //for two hand
                                    Vector3D shoulderright = new Vector3D(skeleton.Joints[JointType.ShoulderRight].Position.X, skeleton.Joints[JointType.ShoulderRight].Position.Y, skeleton.Joints[JointType.ShoulderRight].Position.Z);
                                    Vector3D elbowright = new Vector3D(skeleton.Joints[JointType.ElbowRight].Position.X, skeleton.Joints[JointType.ElbowRight].Position.Y, skeleton.Joints[JointType.ElbowRight].Position.Z);
                                    //vector for high knees

                                   Vector3D ankleleft = new Vector3D(skeleton.Joints[JointType.AnkleLeft].Position.X, skeleton.Joints[JointType.AnkleLeft].Position.Y, skeleton.Joints[JointType.AnkleLeft].Position.Z);
                                   Vector3D kneeleft = new Vector3D(skeleton.Joints[JointType.KneeLeft].Position.X, skeleton.Joints[JointType.KneeLeft].Position.Y, skeleton.Joints[JointType.KneeLeft].Position.Z);
                                   Vector3D hipleft = new Vector3D(skeleton.Joints[JointType.HipLeft].Position.X, skeleton.Joints[JointType.HipLeft].Position.Y, skeleton.Joints[JointType.HipLeft].Position.Z);
                                    //Vector3D SpineShoulder = new Vector3D(skeleton.Joints[JointType.SpineShoulder].Position.X, skeleton.Joints[JointType.SpineShoulder].Position.Y, skeleton.Joints[JointType.SpineShoulder].Position.Z);
                                    LeftElbowAngle = AngleBetweenTwoVectors(elbowl - ShoulderLeft, elbowl - WristLeft); 
                                    rotationAngle = AngleBetweenTwoVectors(elbowl - ShoulderLeft, Neck - Head);
                                    rottwohand=AngleBetweenTwoVectors(kneeleft-ankleleft,kneeleft-hipleft);
                                    footrot=AngleBetweenTwoVectors(ShoulderLeft-elbowl,shoulderright-elbowright);

                                    if (footrot >= 0 && footrot <= 100)
                                    {
                                        if (tempcount4 == 0)
                                        {
                                            countnumber4++;
                                        }

                                        tempcount4 = 1;
                                    }

                                    else
                                    {
                                        if (tempcount4 == 1)
                                        {
                                            // countnumber3++;
                                        }

                                        tempcount4 = 0;
                                    }
                                    if (rottwohand >= 0 && rottwohand <= 120)
                                    {
                                        if (tempcount3 == 0)
                                        {
                                            countnumber3++;
                                        }

                                        tempcount3 = 1;
                                    }
                                    else
                                    {
                                          if (tempcount3 == 1)
                                          {
                                             // countnumber3++;
                                          }
                                   
                                        tempcount3 = 0;
                                    }

                                     
                                     if (LeftElbowAngle >= 0 && LeftElbowAngle <= 90)
                                        {
                                            if (tempcount1 == 0)
                                            {
                                                countnumber1++;
                                            }

                                            tempcount1 = 1;
                                        }

                                        else
                                        {
                                              if (tempcount1 == 1)
                                              {
                                                  //countnumber1++;
                                              }

                                           tempcount1 = 0;
                                        }
                                   
                                   
                                    if (rotationAngle >= 0 && rotationAngle <= 90)
                                    {
                                        if (tempcount2 == 0)
                                        {
                                            countnumber2++;
                                        }

                                        tempcount2 = 1;
                                    }
                                    else
                                    {
                                          if (tempcount2 == 1)
                                          {
                                              countnumber2++;
                                          }
                                   
                                        tempcount2 = 0;
                                    }
                                      
                                   // double angle=AngleBetweenTwoVectors(ElbowLeft, WristLeft);
                                   // Console.WriteLine("angle1 = " + LeftElbowAngle.ToString() + "count1 = " + countnumber1.ToString() + "result1= " + tempcount1.ToString());
                                   // Console.WriteLine("angle2 = " + rotationAngle.ToString() + "count2 = " + countnumber2.ToString() + "result2= " + tempcount2.ToString());

                                }
                            
                                }
                         // Console.WriteLine("angle1 = " + LeftElbowAngle.ToString() + "ElbowRotation= " + countnumber1.ToString());
                          // Console.WriteLine("angle2 = " + rotationAngle.ToString() + "LefthandRotation = " + countnumber2.ToString());
                        //Console.WriteLine("angle3 = " + rottwohand.ToString() + "HighKnees = " + countnumber3.ToString());
                         Console.WriteLine("angle4 = " + footrot.ToString() + "BothHandSwing = " + (countnumber4-1).ToString());
                            Point centerPoint = this.GetPosition2DLocation(depthImageFrame, skeleton.Position);

                            // Scale the skeleton thickness
                            // 1.0 is the desired size at 640 width
                            double scale = this.RenderSize.Width / 640;

                            skeletonCanvas.RefreshSkeleton(skeleton, jointMapping, centerPoint, scale);
                        }

                        if (ImageType == ImageType.Depth)
                        {
                            this.ChooseTrackedSkeletons(this.skeletonData);
                        }
                    }
                }
            }
        }
        public double AngleBetweenTwoVectors(Vector3D vectorA, Vector3D vectorB)
        {
            double dotProduct = 0.0;
            vectorA.Normalize();
            vectorB.Normalize();
            dotProduct = Vector3D.DotProduct(vectorA, vectorB);

            return (double)Math.Acos(dotProduct) / Math.PI * 180;
        }


        private Point GetPosition2DLocation(DepthImageFrame depthFrame, SkeletonPoint skeletonPoint)
        {
            DepthImagePoint depthPoint = depthFrame.MapFromSkeletonPoint(skeletonPoint);

            switch (ImageType)
            {
                case ImageType.Color:
                    ColorImagePoint colorPoint = depthFrame.MapToColorImagePoint(depthPoint.X, depthPoint.Y, this.Kinect.ColorStream.Format);

                    // map back to skeleton.Width & skeleton.Height
                    return new Point(
                        (int)(this.RenderSize.Width * colorPoint.X / this.Kinect.ColorStream.FrameWidth),
                        (int)(this.RenderSize.Height * colorPoint.Y / this.Kinect.ColorStream.FrameHeight));
                case ImageType.Depth:
                    return new Point(
                        (int)(this.RenderSize.Width * depthPoint.X / depthFrame.Width),
                        (int)(this.RenderSize.Height * depthPoint.Y / depthFrame.Height));
                default:
                    throw new ArgumentOutOfRangeException("ImageType was a not expected value: " + ImageType.ToString());
            }
        }

        private void CreateListOfSkeletonCanvases()
        {
            this.skeletonCanvases = new List<KinectSkeleton>
                {
                    this.skeletonCanvas1,
                    this.skeletonCanvas2,
                    this.skeletonCanvas3,
                    this.skeletonCanvas4,
                    this.skeletonCanvas5,
                    this.skeletonCanvas6
                };

            this.skeletonCanvases.ForEach(s => this.jointMappings.Add(new Dictionary<JointType, JointMapping>()));
        }

        // NOTE: The ChooseTrackedSkeletons part of the KinectSkeletonViewer would be useful
        // separate from the SkeletonViewer.
        private void ChooseTrackedSkeletons(IEnumerable<Skeleton> skeletonDataValue)
        {
            switch (TrackingMode)
            {
                case TrackingMode.Closest1Player:
                    this.ChooseClosestSkeletons(skeletonDataValue, 1);
                    break;
                case TrackingMode.Closest2Player:
                    this.ChooseClosestSkeletons(skeletonDataValue, 2);
                    break;
                case TrackingMode.Sticky1Player:
                    this.ChooseOldestSkeletons(skeletonDataValue, 1);
                    break;
                case TrackingMode.Sticky2Player:
                    this.ChooseOldestSkeletons(skeletonDataValue, 2);
                    break;
                case TrackingMode.MostActive1Player:
                    this.ChooseMostActiveSkeletons(skeletonDataValue, 1);
                    break;
                case TrackingMode.MostActive2Player:
                    this.ChooseMostActiveSkeletons(skeletonDataValue, 2);
                    break;
            }
        }

        private void ChooseClosestSkeletons(IEnumerable<Skeleton> skeletonDataValue, int count)
        {
            SortedList<float, int> depthSorted = new SortedList<float, int>();

            foreach (Skeleton s in skeletonDataValue)
            {
                if (s.TrackingState != SkeletonTrackingState.NotTracked)
                {
                    float valueZ = s.Position.Z;
                    while (depthSorted.ContainsKey(valueZ))
                    {
                        valueZ += 0.0001f;
                    }

                    depthSorted.Add(valueZ, s.TrackingId);
                }
            }

            this.ChooseSkeletonsFromList(depthSorted.Values, count);
        }

        private void ChooseOldestSkeletons(IEnumerable<Skeleton> skeletonDataValue, int count)
        {
            List<int> newList = new List<int>();
            
            foreach (Skeleton s in skeletonDataValue)
            {
                if (s.TrackingState != SkeletonTrackingState.NotTracked)
                {
                    newList.Add(s.TrackingId);
                }
            }

            // Remove all elements from the active list that are not currently present
            this.activeList.RemoveAll(k => !newList.Contains(k));

            // Add all elements that aren't already in the activeList
            this.activeList.AddRange(newList.FindAll(k => !this.activeList.Contains(k)));

            this.ChooseSkeletonsFromList(this.activeList, count);
        }

        private void ChooseMostActiveSkeletons(IEnumerable<Skeleton> skeletonDataValue, int count)
        {
            foreach (ActivityWatcher watcher in this.recentActivity)
            {
                watcher.NewPass();
            }

            foreach (Skeleton s in skeletonDataValue)
            {
                if (s.TrackingState != SkeletonTrackingState.NotTracked)
                {
                    ActivityWatcher watcher = this.recentActivity.Find(w => w.TrackingId == s.TrackingId);
                    if (watcher != null)
                    {
                        watcher.Update(s);
                    }
                    else
                    {
                        this.recentActivity.Add(new ActivityWatcher(s));
                    }
                }
            }

            // Remove any skeletons that are gone
            this.recentActivity.RemoveAll(aw => !aw.Updated);

            this.recentActivity.Sort();
            this.ChooseSkeletonsFromList(this.recentActivity.ConvertAll(f => f.TrackingId), count);
        }

        private void ChooseSkeletonsFromList(IList<int> list, int max)
        {
            if (this.Kinect.SkeletonStream.IsEnabled)
            {
                int argCount = Math.Min(list.Count, max);

                if (argCount == 0)
                {
                    this.Kinect.SkeletonStream.ChooseSkeletons();
                }

                if (argCount == 1)
                {
                    this.Kinect.SkeletonStream.ChooseSkeletons(list[0]);
                }

                if (argCount >= 2)
                {
                    this.Kinect.SkeletonStream.ChooseSkeletons(list[0], list[1]);
                }
            }
        }

        private class ActivityWatcher : IComparable<ActivityWatcher>
        {
            private float activityLevel;
            private SkeletonPoint previousPosition;
            private SkeletonPoint previousDelta;

            internal ActivityWatcher(Skeleton s)
            {
                this.activityLevel = 0.0f;
                this.TrackingId = s.TrackingId;
                this.Updated = true;
                this.previousPosition = s.Position;
                this.previousDelta = new SkeletonPoint();
            }

            internal int TrackingId { get; private set; }

            internal bool Updated { get; private set; }

            public int CompareTo(ActivityWatcher other)
            {
                // Use the existing CompareTo on float, but reverse the arguments,
                // since we wish to have larger activityLevels sort ahead of smaller values.
                return other.activityLevel.CompareTo(this.activityLevel);
            }

            internal void NewPass()
            {
                this.Updated = false;
            }

            internal void Update(Skeleton s)
            {
                SkeletonPoint newPosition = s.Position;
                SkeletonPoint newDelta = new SkeletonPoint
                    {
                        X = newPosition.X - this.previousPosition.X,
                        Y = newPosition.Y - this.previousPosition.Y,
                        Z = newPosition.Z - this.previousPosition.Z
                    };

                SkeletonPoint deltaV = new SkeletonPoint
                    {
                        X = newDelta.X - this.previousDelta.X,
                        Y = newDelta.Y - this.previousDelta.Y,
                        Z = newDelta.Z - this.previousDelta.Z
                    };

                this.previousPosition = newPosition;
                this.previousDelta = newDelta;

                float deltaVLengthSquared = (deltaV.X * deltaV.X) + (deltaV.Y * deltaV.Y) + (deltaV.Z * deltaV.Z);
                float deltaVLength = (float)Math.Sqrt(deltaVLengthSquared);

                this.activityLevel = this.activityLevel * ActivityFalloff;
                this.activityLevel += deltaVLength;

                this.Updated = true;
            }
        }
    }
}
