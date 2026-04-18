using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO.Ports;
using System.Windows.Forms;

namespace UnoLedControl
{
    public class AirfoilAnglePage : UserControl
    {
        private readonly SerialPort _serial;
        private readonly TelemetryStore _telemetry;
        private readonly Action<string> _logTx;
        private readonly Timer _timer = new Timer();
        private readonly Timer _sendDebounce = new Timer();

        private bool _connected;

        private TrackBar tbAngle;
        private NumericUpDown nudAngle;
        private TrackBar tbSpeed;
        private Label lblSpeedVal;

        private Button btnSet;
        private Button btnStop;
        private Button btnHome;
        private Button btnSetHome;

        private Label vActual;
        private Label vTarget;
        private Label vError;
        private Label vCmd;

        private double _targetDeg = 0.0;
        private bool _pendingSend;
        private bool _startupSynced = false;

        private int _stepperSps = DefaultSps;

        private readonly Dictionary<Label, string> _last = new Dictionary<Label, string>();

        private const int MinAngleDeg = -180;
        private const int MaxAngleDeg = 180;

        // 1/8 microstep, 200 step motor, direct drive
        private const double StepsPerDegree = 4.444;

        private const int MinSps = 100;
        private const int MaxSps = 3000;
        private const int DefaultSps = 1200;

        private long _lastSentTargetSteps = long.MinValue;

        public AirfoilAnglePage(SerialPort serial, TelemetryStore telemetry, Action<string> logTx)
        {
            _serial = serial;
            _telemetry = telemetry;
            _logTx = logTx;

            Dock = DockStyle.Fill;
            BackColor = Color.Transparent;

            SetStyle(ControlStyles.AllPaintingInWmPaint |
                     ControlStyles.OptimizedDoubleBuffer |
                     ControlStyles.UserPaint, true);
            UpdateStyles();

            BuildUI();

            _timer.Interval = 200;
            _timer.Tick += (s, e) => RefreshValues();
            _timer.Start();

            _sendDebounce.Interval = 150;
            _sendDebounce.Tick += (s, e) =>
            {
                _sendDebounce.Stop();
                if (_pendingSend)
                {
                    _pendingSend = false;
                    SendStepperGotoDeg(_targetDeg, _stepperSps);
                }
            };

            RefreshValues();
        }

        public void SetConnected(bool connected) { _connected = connected; }

        private void BuildUI()
        {
            Controls.Clear();

            var card = new GlassPanel
            {
                FillColor = Color.FromArgb(255, 10, 10, 14),
                CornerRadius = 18,
                Dock = DockStyle.Fill,
                Padding = new Padding(18)
            };
            Controls.Add(card);

            var root = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 2,
                BackColor = Color.Transparent
            };
            root.RowStyles.Add(new RowStyle(SizeType.Absolute, 78));
            root.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            card.Controls.Add(root);

            var header = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 3,
                BackColor = Color.Transparent
            };
            header.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 55f));
            header.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 30f));
            header.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 15f));
            root.Controls.Add(header, 0, 0);

            var title = new Label
            {
                Text = "Airfoil Angle",
                ForeColor = Color.White,
                BackColor = Color.Transparent,
                Font = new Font("Segoe UI", 16f, FontStyle.Bold),
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleLeft
            };
            header.Controls.Add(title, 0, 0);

            var mid = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                BackColor = Color.Transparent
            };
            mid.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 140));
            mid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            header.Controls.Add(mid, 1, 0);

            var lblTarget = new Label
            {
                Text = "Target (deg)",
                ForeColor = Color.Gainsboro,
                BackColor = Color.Transparent,
                Font = new Font("Segoe UI", 10f, FontStyle.Bold),
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleLeft
            };
            mid.Controls.Add(lblTarget, 0, 0);

            nudAngle = new NumericUpDown
            {
                Minimum = MinAngleDeg,
                Maximum = MaxAngleDeg,
                DecimalPlaces = 1,
                Increment = 0.5M,
                Value = 0,
                Dock = DockStyle.Fill,
                Margin = new Padding(0, 18, 0, 18),
                BackColor = Color.FromArgb(25, 25, 35),
                ForeColor = Color.White
            };
            nudAngle.ValueChanged += (s, e) =>
            {
                var v = (double)nudAngle.Value;
                SetTarget(v, sendNow: true);
            };
            mid.Controls.Add(nudAngle, 1, 0);

            btnStop = new Button
            {
                Text = "STOP",
                Size = new Size(110, 36),
                Anchor = AnchorStyles.Right | AnchorStyles.Top,
                Margin = new Padding(0, 18, 0, 18)
            };
            StyleButton(btnStop);
            btnStop.Click += (s, e) =>
            {
                Send("STEPPER STOP");
                SetIfChanged(vCmd, "STEPPER STOP");
            };
            header.Controls.Add(btnStop, 2, 0);

            var scroll = new Panel
            {
                Dock = DockStyle.Fill,
                AutoScroll = true,
                BackColor = Color.Transparent
            };
            root.Controls.Add(scroll, 0, 1);

            var body = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                ColumnCount = 1,
                BackColor = Color.Transparent,
                Padding = new Padding(0, 6, 0, 0)
            };
            scroll.Controls.Add(body);

            body.Controls.Add(MakeSectionHeader("Angle control"));

            var controlGrid = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                ColumnCount = 4,
                BackColor = Color.Transparent,
                Margin = new Padding(0, 8, 0, 6)
            };
            controlGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 140));
            controlGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 80));
            controlGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 520));
            controlGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 320));
            body.Controls.Add(controlGrid);

            controlGrid.RowCount = 2;
            controlGrid.RowStyles.Add(new RowStyle(SizeType.Absolute, 72));
            controlGrid.RowStyles.Add(new RowStyle(SizeType.Absolute, 72));

            var lbl = new Label
            {
                Text = "Angle",
                ForeColor = Color.Gainsboro,
                BackColor = Color.Transparent,
                Font = new Font("Segoe UI", 10f, FontStyle.Bold),
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleLeft,
                Margin = new Padding(0, 8, 0, 8)
            };
            controlGrid.Controls.Add(lbl, 0, 0);

            var lblVal = new Label
            {
                Text = "0.0°",
                ForeColor = Color.White,
                BackColor = Color.Transparent,
                Font = new Font("Consolas", 12f, FontStyle.Bold),
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleCenter,
                Margin = new Padding(0, 8, 0, 8)
            };
            controlGrid.Controls.Add(lblVal, 1, 0);

            tbAngle = new TrackBar
            {
                Minimum = MinAngleDeg * 10,
                Maximum = MaxAngleDeg * 10,
                TickFrequency = 10,
                LargeChange = 5,
                SmallChange = 1,
                Anchor = AnchorStyles.Left | AnchorStyles.Right,
                Height = 45,
                Margin = new Padding(6, 14, 6, 14)
            };
            tbAngle.Scroll += (s, e) =>
            {
                var deg = tbAngle.Value / 10.0;
                lblVal.Text = deg.ToString("0.0", CultureInfo.InvariantCulture) + "°";
                SetTarget(deg, sendNow: true);
            };
            controlGrid.Controls.Add(tbAngle, 2, 0);

            var btnGrid = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.LeftToRight,
                WrapContents = false,
                BackColor = Color.Transparent,
                Margin = new Padding(0, 14, 0, 14)
            };
            controlGrid.Controls.Add(btnGrid, 3, 0);

            btnSet = new Button { Text = "SET", Size = new Size(70, 36) };
            StyleButton(btnSet);
            btnSet.Click += (s, e) => SendStepperGotoDeg(_targetDeg, _stepperSps);
            btnGrid.Controls.Add(btnSet);

            btnHome = new Button { Text = "HOME", Size = new Size(80, 36) };
            StyleButton(btnHome);
            btnHome.Click += (s, e) =>
            {
                _targetDeg = 0.0;
                _lastSentTargetSteps = long.MinValue;
                Send("STEPPER GOTO 0 " + _stepperSps);
                SetIfChanged(vCmd, "STEPPER GOTO 0 " + _stepperSps);
                SetTarget(0.0, sendNow: false);
            };
            btnGrid.Controls.Add(btnHome);

            btnSetHome = new Button { Text = "SET HOME", Size = new Size(110, 36) };
            StyleButton(btnSetHome);
            btnSetHome.Click += (s, e) =>
            {
                Send("ENCODER ZERO");
                SetIfChanged(vCmd, "ENCODER ZERO");

                _startupSynced = false;
                _targetDeg = 0.0;
                _lastSentTargetSteps = 0;
                SetTarget(0.0, sendNow: false);
            };
            btnGrid.Controls.Add(btnSetHome);

            var lblSpeed = new Label
            {
                Text = "Speed (sps)",
                ForeColor = Color.Gainsboro,
                BackColor = Color.Transparent,
                Font = new Font("Segoe UI", 10f, FontStyle.Bold),
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleLeft,
                Margin = new Padding(0, 8, 0, 8)
            };
            controlGrid.Controls.Add(lblSpeed, 0, 1);

            lblSpeedVal = new Label
            {
                Text = _stepperSps.ToString(CultureInfo.InvariantCulture),
                ForeColor = Color.White,
                BackColor = Color.Transparent,
                Font = new Font("Consolas", 12f, FontStyle.Bold),
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.MiddleCenter,
                Margin = new Padding(0, 8, 0, 8)
            };
            controlGrid.Controls.Add(lblSpeedVal, 1, 1);

            tbSpeed = new TrackBar
            {
                Minimum = MinSps,
                Maximum = MaxSps,
                TickFrequency = 200,
                LargeChange = 100,
                SmallChange = 50,
                Value = DefaultSps,
                Anchor = AnchorStyles.Left | AnchorStyles.Right,
                Height = 45,
                Margin = new Padding(6, 14, 6, 14)
            };
            tbSpeed.Scroll += (s, e) =>
            {
                _stepperSps = tbSpeed.Value;
                lblSpeedVal.Text = _stepperSps.ToString(CultureInfo.InvariantCulture);
            };
            controlGrid.Controls.Add(tbSpeed, 2, 1);

            body.Controls.Add(MakeSectionHeader("Live feedback"));

            var live = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowAndShrink,
                ColumnCount = 2,
                BackColor = Color.Transparent
            };
            live.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 65f));
            live.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 35f));
            body.Controls.Add(live);

            vActual = AddRow(live, "AS5600 actual angle (deg)", "--", chip: true);
            vTarget = AddRow(live, "Target angle (deg)", "--", chip: true);
            vError = AddRow(live, "Error (Target - Actual)", "--", chip: true);
            vCmd = AddRow(live, "Last stepper cmd", "--", chip: false);

            _last.Clear();
            _last[vActual] = null;
            _last[vTarget] = null;
            _last[vError] = null;
            _last[vCmd] = null;
        }

        private Control MakeSectionHeader(string text)
        {
            return new Label
            {
                Text = text,
                ForeColor = Color.White,
                BackColor = Color.FromArgb(25, 25, 35),
                Dock = DockStyle.Top,
                Font = new Font("Segoe UI", 10f, FontStyle.Bold),
                TextAlign = ContentAlignment.MiddleLeft,
                Padding = new Padding(10, 0, 0, 0),
                Height = 30,
                Margin = new Padding(0, 10, 0, 2)
            };
        }

        private Label AddRow(TableLayoutPanel table, string name, string initial, bool chip)
        {
            int row = table.RowCount++;
            table.RowStyles.Add(new RowStyle(SizeType.Absolute, 36));

            var k = new Label
            {
                Text = name,
                ForeColor = Color.Gainsboro,
                BackColor = Color.Transparent,
                Dock = DockStyle.Fill,
                Font = new Font("Segoe UI", 10f, FontStyle.Bold),
                TextAlign = ContentAlignment.MiddleLeft,
                Margin = new Padding(0, 2, 0, 2)
            };

            Label v = new Label
            {
                Text = initial,
                ForeColor = Color.Lime,
                BackColor = chip ? Color.FromArgb(10, 10, 12) : Color.Transparent,
                Dock = DockStyle.Fill,
                Font = new Font("Consolas", chip ? 14f : 11f, FontStyle.Bold),
                TextAlign = chip ? ContentAlignment.MiddleCenter : ContentAlignment.MiddleLeft,
                Margin = new Padding(8, 2, 0, 2)
            };

            table.Controls.Add(k, 0, row);
            table.Controls.Add(v, 1, row);
            return v;
        }

        private void StyleButton(Button b)
        {
            b.FlatStyle = FlatStyle.Flat;
            b.FlatAppearance.BorderSize = 1;
            b.FlatAppearance.BorderColor = Color.FromArgb(70, 255, 255, 255);
            b.BackColor = Color.FromArgb(25, 25, 35);
            b.ForeColor = Color.White;
            b.Font = new Font("Segoe UI", 10f, FontStyle.Bold);
            b.Cursor = Cursors.Hand;
        }

        private void SetTarget(double deg, bool sendNow)
        {
            deg = Math.Max(MinAngleDeg, Math.Min(MaxAngleDeg, deg));
            _targetDeg = deg;

            int tbVal = (int)Math.Round(deg * 10.0);
            if (tbAngle != null && tbAngle.Value != tbVal)
                tbAngle.Value = tbVal;

            decimal nudVal = (decimal)deg;
            if (nudAngle != null && nudAngle.Value != nudVal)
                nudAngle.Value = nudVal;

            if (sendNow)
            {
                _pendingSend = true;
                _sendDebounce.Stop();
                _sendDebounce.Start();
            }
        }

        private long DegreesToSteps(double deg)
        {
            return (long)Math.Round(deg * StepsPerDegree);
        }

        private void SendStepperGotoDeg(double deg, int sps)
        {
            long targetSteps = DegreesToSteps(deg);

            if (targetSteps == _lastSentTargetSteps)
                return;

            _lastSentTargetSteps = targetSteps;

            string cmd = $"STEPPER GOTO {targetSteps} {sps}";
            Send(cmd);
            SetIfChanged(vCmd, cmd);
        }

        private void Send(string cmd)
        {
            if (!_connected || _serial == null || !_serial.IsOpen) return;
            try
            {
                _serial.WriteLine(cmd);
                _logTx?.Invoke(cmd);
            }
            catch { }
        }

        private void SetIfChanged(Label label, string value)
        {
            if (label == null) return;
            if (!_last.TryGetValue(label, out var lastVal) || lastVal != value)
            {
                _last[label] = value;
                label.Text = value;
            }
        }

        private void RefreshValues()
        {
            string asStr = _telemetry.Get("as5600", "--");
            SetIfChanged(vActual, asStr);

            if (TryParseDouble(asStr, out var actual))
            {
                if (!_startupSynced)
                {
                    _startupSynced = true;
                    _targetDeg = actual;

                    int tbVal = (int)Math.Round(actual * 10.0);
                    tbVal = Math.Max(tbAngle.Minimum, Math.Min(tbAngle.Maximum, tbVal));
                    if (tbAngle.Value != tbVal)
                        tbAngle.Value = tbVal;

                    decimal nudVal = (decimal)Math.Max(MinAngleDeg, Math.Min(MaxAngleDeg, actual));
                    if (nudAngle.Value != nudVal)
                        nudAngle.Value = nudVal;

                    _lastSentTargetSteps = DegreesToSteps(actual);
                }

                SetIfChanged(vTarget, _targetDeg.ToString("0.0", CultureInfo.InvariantCulture));

                double err = _targetDeg - actual;
                SetIfChanged(vError, err.ToString("0.0", CultureInfo.InvariantCulture));
            }
            else
            {
                SetIfChanged(vTarget, _targetDeg.ToString("0.0", CultureInfo.InvariantCulture));
                SetIfChanged(vError, "--");
            }
        }

        private bool TryParseDouble(string s, out double v)
        {
            v = 0;
            if (string.IsNullOrWhiteSpace(s)) return false;
            s = s.Trim().Replace(',', '.');
            return double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out v);
        }
    }
}