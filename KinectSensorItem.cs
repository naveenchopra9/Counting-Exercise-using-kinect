//------------------------------------------------------------------------------
// <copyright file="KinectSensorItem.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.KinectExplorer
{
    using System.ComponentModel;
    using Microsoft.Kinect;

    public class KinectSensorItem : INotifyPropertyChanged
    {
        private KinectStatus status;
        private KinectWindow window;

        #region INotifyPropertyChanged Members

        public event PropertyChangedEventHandler PropertyChanged;

        #endregion

        public KinectSensor Sensor { get; set; }

        public KinectStatus Status
        {
            get
            {
                return this.status;
            }

            set
            {
                this.status = value;
                this.NotifyPropertyChanged("Status");
            }
        }

        public KinectWindow Window
        {
            get
            {
                return this.window;
            }

            set
            {
                this.window = value;
                this.NotifyPropertyChanged("Window");
                if (this.window != null)
                {
                    this.window.DataContext = this;
                }
            }
        }

        private void NotifyPropertyChanged(string propertyName)
        {
            if (this.PropertyChanged != null)
            {
                this.PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
            }
        }
    }
}
