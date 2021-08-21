using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Windows.Forms;

namespace _3d
{
    public partial class Waiter : Form
    {
        public Waiter()
        {
            InitializeComponent();
            Shown += Waiter_Shown;
        }

        public string Title
        {
            get { return Text; }
            set
            {
                Text = value;
            }
        }
        private void Waiter_Shown(object sender, EventArgs e)
        {
            bw.RunWorkerAsync();
        }
        BackgroundWorker bw = new BackgroundWorker();
        public void Progress(int v)
        {
            progressBar1.Invoke((Action)(() =>
            {
                progressBar1.Value = v;
            }));
        }
        internal void Init(Action act)
        {

            bw.DoWork += (s, e) =>
            {
                act();
            };
            bw.RunWorkerCompleted += Bw_RunWorkerCompleted;
        }

        private void Bw_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            Close();
        }
    }
}
