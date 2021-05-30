using System;
using System.Globalization;
using System.Windows.Forms;

namespace _3d
{
    public partial class LoadParams : Form
    {
        public LoadParams()
        {
            InitializeComponent();
            radioButton5.Checked = true;
        }

        public enum ScaleModeEnum
        {
            Disabled, Manual, Auto
        }
        public ScaleModeEnum ScaleMode { get; set; }
        public bool Centralized { get; set; }
        
        public double ScaleKoef { get; set; } = 1;
        private void button2_Click(object sender, EventArgs e)
        {
            Centralized = checkBox1.Checked;
            Close();
        }



        private void textBox1_TextChanged(object sender, EventArgs e)
        {
            try
            {
                ScaleKoef = double.Parse(textBox1.Text.Replace(",", "."), CultureInfo.InvariantCulture);
            }
            catch (Exception ex)
            {

            }
        }

        private void radioButton4_CheckedChanged(object sender, EventArgs e)
        {
            if (radioButton4.Checked)
            {
                ScaleMode = ScaleModeEnum.Manual;
            }

        }

        private void radioButton5_CheckedChanged(object sender, EventArgs e)
        {
            if (radioButton5.Checked)
            {
                ScaleMode = ScaleModeEnum.Auto;
            }
        }
    }
}
