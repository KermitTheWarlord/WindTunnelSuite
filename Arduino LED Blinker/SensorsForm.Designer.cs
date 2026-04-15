namespace UnoLedControl
{
    partial class SensorsForm
    {
        private System.ComponentModel.IContainer components = null;

        private UnoLedControl.GlassPanel panelGlass;
        private System.Windows.Forms.Label lblTitle;

        // Keys
        private System.Windows.Forms.Label lblLastRxK, lblFanAllK, lblFan1K, lblFan2K, lblFan3K, lblFan4K;
        private System.Windows.Forms.Label lblRpm1K, lblRpm2K, lblRpm3K, lblRpm4K;
        private System.Windows.Forms.Label lblMpxAdcK, lblBmpPK, lblBmpTK, lblAs5600K;

        // Values
        private System.Windows.Forms.Label lblLastRxV, lblFanAllV, lblFan1V, lblFan2V, lblFan3V, lblFan4V;
        private System.Windows.Forms.Label lblRpm1V, lblRpm2V, lblRpm3V, lblRpm4V;
        private System.Windows.Forms.Label lblMpxAdcV, lblBmpPV, lblBmpTV, lblAs5600V;

        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
                components.Dispose();
            base.Dispose(disposing);
        }

        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();

            this.panelGlass = new UnoLedControl.GlassPanel();
            this.lblTitle = new System.Windows.Forms.Label();

            // Create labels
            this.lblLastRxK = new System.Windows.Forms.Label();
            this.lblFanAllK = new System.Windows.Forms.Label();
            this.lblFan1K = new System.Windows.Forms.Label();
            this.lblFan2K = new System.Windows.Forms.Label();
            this.lblFan3K = new System.Windows.Forms.Label();
            this.lblFan4K = new System.Windows.Forms.Label();

            this.lblRpm1K = new System.Windows.Forms.Label();
            this.lblRpm2K = new System.Windows.Forms.Label();
            this.lblRpm3K = new System.Windows.Forms.Label();
            this.lblRpm4K = new System.Windows.Forms.Label();

            this.lblMpxAdcK = new System.Windows.Forms.Label();
            this.lblBmpPK = new System.Windows.Forms.Label();
            this.lblBmpTK = new System.Windows.Forms.Label();
            this.lblAs5600K = new System.Windows.Forms.Label();

            this.lblLastRxV = new System.Windows.Forms.Label();
            this.lblFanAllV = new System.Windows.Forms.Label();
            this.lblFan1V = new System.Windows.Forms.Label();
            this.lblFan2V = new System.Windows.Forms.Label();
            this.lblFan3V = new System.Windows.Forms.Label();
            this.lblFan4V = new System.Windows.Forms.Label();

            this.lblRpm1V = new System.Windows.Forms.Label();
            this.lblRpm2V = new System.Windows.Forms.Label();
            this.lblRpm3V = new System.Windows.Forms.Label();
            this.lblRpm4V = new System.Windows.Forms.Label();

            this.lblMpxAdcV = new System.Windows.Forms.Label();
            this.lblBmpPV = new System.Windows.Forms.Label();
            this.lblBmpTV = new System.Windows.Forms.Label();
            this.lblAs5600V = new System.Windows.Forms.Label();

            this.SuspendLayout();

            // Form
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(760, 420);
            this.Name = "SensorsForm";
            this.Text = "Sensors Output";

            // Glass panel
            this.panelGlass.Location = new System.Drawing.Point(12, 12);
            this.panelGlass.Size = new System.Drawing.Size(736, 396);
            this.panelGlass.Name = "panelGlass";
            this.panelGlass.CornerRadius = 16;
            this.panelGlass.FillColor = System.Drawing.Color.FromArgb(190, 10, 10, 14);

            // Title
            this.lblTitle.Location = new System.Drawing.Point(16, 14);
            this.lblTitle.Size = new System.Drawing.Size(300, 28);
            this.lblTitle.Text = "Live Telemetry";
            this.lblTitle.BackColor = System.Drawing.Color.Transparent;
            this.lblTitle.ForeColor = System.Drawing.Color.White;
            this.lblTitle.Font = new System.Drawing.Font("Segoe UI", 14f, System.Drawing.FontStyle.Bold);

            // Layout
            int xK = 24, xV = 360;
            int y = 58;
            int dy = 22;

            SetupRow(lblLastRxK, lblLastRxV, xK, xV, y, "Last RX"); y += dy;

            SetupRow(lblFanAllK, lblFanAllV, xK, xV, y, "Fan ALL (%)"); y += dy;
            SetupRow(lblFan1K, lblFan1V, xK, xV, y, "Fan1 (%)"); y += dy;
            SetupRow(lblFan2K, lblFan2V, xK, xV, y, "Fan2 (%)"); y += dy;
            SetupRow(lblFan3K, lblFan3V, xK, xV, y, "Fan3 (%)"); y += dy;
            SetupRow(lblFan4K, lblFan4V, xK, xV, y, "Fan4 (%)"); y += dy;

            y += 6;

            SetupRow(lblRpm1K, lblRpm1V, xK, xV, y, "RPM1"); y += dy;
            SetupRow(lblRpm2K, lblRpm2V, xK, xV, y, "RPM2"); y += dy;
            SetupRow(lblRpm3K, lblRpm3V, xK, xV, y, "RPM3"); y += dy;
            SetupRow(lblRpm4K, lblRpm4V, xK, xV, y, "RPM4"); y += dy;

            y += 6;

            SetupRow(lblMpxAdcK, lblMpxAdcV, xK, xV, y, "MPXV7002 (mBar)"); y += dy;
            SetupRow(lblBmpPK, lblBmpPV, xK, xV, y, "BMP280 Pressure (hPa)"); y += dy;
            SetupRow(lblBmpTK, lblBmpTV, xK, xV, y, "BMP280 Temp (°C)"); y += dy;
            SetupRow(lblAs5600K, lblAs5600V, xK, xV, y, "AS5600 Angle (deg)"); y += dy;

            // Add controls
            this.panelGlass.Controls.Add(this.lblTitle);

            AddPair(lblLastRxK, lblLastRxV);

            AddPair(lblFanAllK, lblFanAllV);
            AddPair(lblFan1K, lblFan1V);
            AddPair(lblFan2K, lblFan2V);
            AddPair(lblFan3K, lblFan3V);
            AddPair(lblFan4K, lblFan4V);

            AddPair(lblRpm1K, lblRpm1V);
            AddPair(lblRpm2K, lblRpm2V);
            AddPair(lblRpm3K, lblRpm3V);
            AddPair(lblRpm4K, lblRpm4V);

            AddPair(lblMpxAdcK, lblMpxAdcV);
            AddPair(lblBmpPK, lblBmpPV);
            AddPair(lblBmpTK, lblBmpTV);
            AddPair(lblAs5600K, lblAs5600V);

            this.Controls.Add(this.panelGlass);

            this.ResumeLayout(false);
        }

        private void AddPair(System.Windows.Forms.Label k, System.Windows.Forms.Label v)
        {
            this.panelGlass.Controls.Add(k);
            this.panelGlass.Controls.Add(v);
        }

        private void SetupRow(System.Windows.Forms.Label k, System.Windows.Forms.Label v, int xK, int xV, int y, string keyText)
        {
            k.Location = new System.Drawing.Point(xK, y);
            k.Size = new System.Drawing.Size(320, 20);
            k.Text = keyText;
            k.BackColor = System.Drawing.Color.Transparent;

            v.Location = new System.Drawing.Point(xV, y);
            v.Size = new System.Drawing.Size(320, 20);
            v.Text = "--";
            v.BackColor = System.Drawing.Color.Transparent;
        }
    }
}
