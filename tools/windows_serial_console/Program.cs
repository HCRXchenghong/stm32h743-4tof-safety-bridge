using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Json;
using System.Text;
using System.Text.RegularExpressions;
using System.Windows.Forms;

namespace H7TofSerialConsole
{
    [DataContract]
    internal sealed class AppSettings
    {
        [DataMember] public string PortName = string.Empty;
        [DataMember] public int BaudRate = 115200;
        [DataMember] public int BaselineMm = 391;
        [DataMember] public int ToleranceMm = 28;
        [DataMember] public int ReleaseHoldMs = 3000;
        [DataMember] public bool AutoReconnect = false;
    }

    internal sealed class PendingApply
    {
        public int BaselineMm;
        public int ToleranceMm;
        public DateTime RequestedAtUtc;
    }

    internal sealed class H7Frame
    {
        public int Sequence;
        public int[] Tof = new int[4];
        public int Estop;
        public int ValidMask;
        public int FaultMask;
        public int TripMask;
        public int ActiveMask;
        public int SelfEstop;
        public int ExternalEstop;
        public int BaselineMm = 391;
        public int ToleranceMm = 28;
        public int ThresholdMm = 419;
        public int ReleaseRemainingMs;
        public bool HasBoardParameters;
        public bool HasBoardThreshold;
    }

    internal sealed class MainForm : Form
    {
        private const int DefaultBaselineMm = 391;
        private const int DefaultToleranceMm = 28;
        private const int DefaultReleaseHoldMs = 3000;

        private readonly SerialPort _serialPort = new SerialPort();
        private readonly StringBuilder _readBuffer = new StringBuilder();
        private readonly Timer _uiTimer = new Timer();
        private readonly object _sync = new object();
        private readonly string _settingsPath;
        private AppSettings _settings;
        private PendingApply _pendingApply;
        private bool _hasBoardParameterSync;
        private bool _warnedMissingBoardParameters;
        private bool _parameterInputsDirty;
        private bool _suppressParameterInputTracking;
        private int _lastBoardBaselineMm = DefaultBaselineMm;
        private int _lastBoardToleranceMm = DefaultToleranceMm;
        private DateTime _lastFrameUtc = DateTime.MinValue;
        private long _lineCounter;

        private ComboBox _portBox;
        private ComboBox _baudBox;
        private Button _refreshPortsButton;
        private Button _connectButton;
        private Button _disconnectButton;
        private CheckBox _autoReconnectBox;
        private Label _connectionLabel;
        private NumericUpDown _baselineInput;
        private NumericUpDown _toleranceInput;
        private NumericUpDown _releaseHoldInput;
        private Button _applyParamsButton;
        private Button _releaseButton;
        private Label _applyStatusLabel;
        private Label _estopBannerLabel;
        private Label _estopDetailLabel;
        private RichTextBox _logBox;
        private readonly Dictionary<string, Label> _valueLabels = new Dictionary<string, Label>();
        private readonly Label[] _tofLabels = new Label[4];

        public MainForm()
        {
            Text = "H7 TOF Safety Console";
            StartPosition = FormStartPosition.CenterScreen;
            MinimumSize = new Size(1200, 820);
            Size = new Size(1320, 900);
            Font = new Font("Microsoft YaHei UI", 9F, FontStyle.Regular, GraphicsUnit.Point);
            BackColor = Color.FromArgb(15, 24, 36);
            ForeColor = Color.FromArgb(236, 244, 255);

            string appDir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "H7TofSafetyBridge");
            Directory.CreateDirectory(appDir);
            _settingsPath = Path.Combine(appDir, "desktop-settings.json");
            _settings = LoadSettings();

            _serialPort.NewLine = "\n";
            _serialPort.Encoding = Encoding.ASCII;
            _serialPort.ReadTimeout = 500;
            _serialPort.WriteTimeout = 1000;
            _serialPort.DataReceived += SerialPortOnDataReceived;
            _serialPort.ErrorReceived += SerialPortOnErrorReceived;

            BuildUi();
            ApplySettingsToUi();
            RefreshPortList();
            ResetDashboard();

            _uiTimer.Interval = 750;
            _uiTimer.Tick += UiTimerOnTick;
            _uiTimer.Start();

            FormClosing += OnFormClosing;
            Shown += delegate
            {
                AppendLog("桌面工具已启动。建议先连接串口，再观察板端回读参数是否确认。", Color.LightSkyBlue);
            };
        }

        private void BuildUi()
        {
            var root = new TableLayoutPanel();
            root.Dock = DockStyle.Fill;
            root.ColumnCount = 1;
            root.RowCount = 4;
            root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            root.RowStyles.Add(new RowStyle(SizeType.Percent, 55F));
            root.RowStyles.Add(new RowStyle(SizeType.Percent, 45F));
            root.Padding = new Padding(14);
            Controls.Add(root);

            root.Controls.Add(BuildConnectionPanel(), 0, 0);
            root.Controls.Add(BuildCommandPanel(), 0, 1);
            root.Controls.Add(BuildStatusPanel(), 0, 2);
            root.Controls.Add(BuildLogPanel(), 0, 3);
        }

        private Control BuildConnectionPanel()
        {
            var panel = CreatePanel();
            panel.Padding = new Padding(14);
            panel.Height = 110;

            var layout = new TableLayoutPanel();
            layout.Dock = DockStyle.Fill;
            layout.ColumnCount = 8;
            layout.RowCount = 2;
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 110F));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 190F));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 110F));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 130F));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 110F));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 110F));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 140F));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
            panel.Controls.Add(layout);

            layout.Controls.Add(CreateCaption("串口"), 0, 0);
            _portBox = CreateComboBox();
            layout.Controls.Add(_portBox, 1, 0);

            _refreshPortsButton = CreateButton("刷新串口", OnRefreshPorts);
            layout.Controls.Add(_refreshPortsButton, 2, 0);

            layout.Controls.Add(CreateCaption("波特率"), 3, 0);
            _baudBox = CreateComboBox();
            _baudBox.Items.AddRange(new object[] { "115200", "230400", "460800", "921600" });
            layout.Controls.Add(_baudBox, 4, 0);

            _connectButton = CreateButton("连接", OnConnect);
            layout.Controls.Add(_connectButton, 5, 0);

            _disconnectButton = CreateButton("断开", OnDisconnect);
            _disconnectButton.Enabled = false;
            layout.Controls.Add(_disconnectButton, 6, 0);

            _autoReconnectBox = new CheckBox();
            _autoReconnectBox.Text = "自动重连";
            _autoReconnectBox.ForeColor = ForeColor;
            _autoReconnectBox.BackColor = Color.Transparent;
            _autoReconnectBox.AutoSize = true;
            _autoReconnectBox.Margin = new Padding(4, 12, 4, 4);
            layout.Controls.Add(_autoReconnectBox, 1, 1);

            _connectionLabel = new Label();
            _connectionLabel.AutoSize = true;
            _connectionLabel.Font = new Font(Font, FontStyle.Bold);
            _connectionLabel.Margin = new Padding(8, 12, 4, 4);
            _connectionLabel.Text = "未连接";
            layout.Controls.Add(_connectionLabel, 3, 1);
            layout.SetColumnSpan(_connectionLabel, 4);

            return panel;
        }

        private Control BuildCommandPanel()
        {
            var panel = CreatePanel();
            panel.Padding = new Padding(14);
            panel.Height = 124;

            var layout = new TableLayoutPanel();
            layout.Dock = DockStyle.Fill;
            layout.ColumnCount = 7;
            layout.RowCount = 2;
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 150F));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 140F));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 150F));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 140F));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 140F));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 120F));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
            panel.Controls.Add(layout);

            layout.Controls.Add(CreateCaption("Baseline (mm)"), 0, 0);
            _baselineInput = CreateNumericUpDown(0, 65534, DefaultBaselineMm);
            _baselineInput.ValueChanged += OnParameterInputValueChanged;
            layout.Controls.Add(_baselineInput, 1, 0);

            layout.Controls.Add(CreateCaption("Tolerance (mm)"), 2, 0);
            _toleranceInput = CreateNumericUpDown(0, 65534, DefaultToleranceMm);
            _toleranceInput.ValueChanged += OnParameterInputValueChanged;
            layout.Controls.Add(_toleranceInput, 3, 0);

            _applyParamsButton = CreateButton("下发参数", OnApplyParameters);
            layout.Controls.Add(_applyParamsButton, 4, 0);

            _releaseButton = CreateButton("恢复急停", OnReleaseEstop);
            layout.Controls.Add(_releaseButton, 5, 0);

            _applyStatusLabel = new Label();
            _applyStatusLabel.AutoSize = true;
            _applyStatusLabel.Margin = new Padding(8, 12, 4, 4);
            _applyStatusLabel.Text = "未下发";
            layout.Controls.Add(_applyStatusLabel, 6, 0);

            layout.Controls.Add(CreateCaption("人工解除(ms)"), 0, 1);
            _releaseHoldInput = CreateNumericUpDown(1, 600000, DefaultReleaseHoldMs);
            layout.Controls.Add(_releaseHoldInput, 1, 1);

            var note = new Label();
            note.Text = "纯 TOF 模式只保留参数下发和人工解除。参数允许下发，但只有收到板端 B/T/TH 回读后才算确认成功。";
            note.ForeColor = Color.FromArgb(150, 172, 196);
            note.AutoSize = true;
            note.Margin = new Padding(8, 12, 4, 4);
            layout.Controls.Add(note, 2, 1);
            layout.SetColumnSpan(note, 5);

            return panel;
        }
        private Control BuildStatusPanel()
        {
            var split = new TableLayoutPanel();
            split.Dock = DockStyle.Fill;
            split.ColumnCount = 2;
            split.RowCount = 1;
            split.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 58F));
            split.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 42F));

            split.Controls.Add(BuildSensorPanel(), 0, 0);
            split.Controls.Add(BuildTelemetryPanel(), 1, 0);
            return split;
        }

        private Control BuildSensorPanel()
        {
            var panel = CreatePanel();
            panel.Padding = new Padding(14);
            panel.Margin = new Padding(0, 0, 8, 0);

            var title = CreateSectionTitle("TOF 距离");
            panel.Controls.Add(title);

            var grid = new TableLayoutPanel();
            grid.Dock = DockStyle.Fill;
            grid.ColumnCount = 2;
            grid.RowCount = 2;
            grid.Padding = new Padding(0, 34, 0, 0);
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
            grid.RowStyles.Add(new RowStyle(SizeType.Percent, 50F));
            grid.RowStyles.Add(new RowStyle(SizeType.Percent, 50F));
            panel.Controls.Add(grid);

            string[] labels = { "TOF1 右前", "TOF2 右后", "TOF3 左后", "TOF4 左前" };
            for (int i = 0; i < 4; i++)
            {
                var card = CreatePanel();
                card.Padding = new Padding(12);
                card.Margin = new Padding(6);
                card.Dock = DockStyle.Fill;

                var stack = new TableLayoutPanel();
                stack.Dock = DockStyle.Fill;
                stack.RowCount = 3;
                stack.ColumnCount = 1;
                stack.RowStyles.Add(new RowStyle(SizeType.AutoSize));
                stack.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
                stack.RowStyles.Add(new RowStyle(SizeType.AutoSize));
                card.Controls.Add(stack);

                var name = new Label();
                name.Text = labels[i];
                name.AutoSize = true;
                name.ForeColor = Color.FromArgb(132, 156, 182);
                stack.Controls.Add(name, 0, 0);

                var value = new Label();
                value.Text = "--";
                value.Font = new Font("Consolas", 28F, FontStyle.Bold, GraphicsUnit.Point);
                value.AutoSize = false;
                value.Dock = DockStyle.Fill;
                value.Margin = new Padding(0, 10, 0, 0);
                value.TextAlign = ContentAlignment.MiddleLeft;
                stack.Controls.Add(value, 0, 1);
                _tofLabels[i] = value;

                var unit = new Label();
                unit.Text = "mm";
                unit.AutoSize = true;
                unit.ForeColor = Color.FromArgb(132, 156, 182);
                stack.Controls.Add(unit, 0, 2);

                grid.Controls.Add(card, i % 2, i / 2);
            }

            return panel;
        }

        private Control BuildTelemetryPanel()
        {
            var panel = CreatePanel();
            panel.Padding = new Padding(14);
            panel.Margin = new Padding(8, 0, 0, 0);

            var title = CreateSectionTitle("状态与参数");
            panel.Controls.Add(title);

            var content = new TableLayoutPanel();
            content.Dock = DockStyle.Fill;
            content.Padding = new Padding(0, 34, 0, 0);
            content.ColumnCount = 1;
            content.RowCount = 2;
            content.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            content.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
            panel.Controls.Add(content);

            var estopCard = CreatePanel();
            estopCard.Dock = DockStyle.Top;
            estopCard.Height = 94;
            estopCard.Margin = new Padding(0, 0, 0, 10);
            estopCard.Padding = new Padding(14, 12, 14, 12);

            var estopLayout = new TableLayoutPanel();
            estopLayout.Dock = DockStyle.Fill;
            estopLayout.ColumnCount = 1;
            estopLayout.RowCount = 2;
            estopLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            estopLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            estopCard.Controls.Add(estopLayout);

            _estopBannerLabel = new Label();
            _estopBannerLabel.AutoSize = true;
            _estopBannerLabel.Font = new Font(Font.FontFamily, 18F, FontStyle.Bold, GraphicsUnit.Point);
            _estopBannerLabel.Text = "当前状态：未急停";
            _estopBannerLabel.ForeColor = Color.FromArgb(103, 240, 169);
            estopLayout.Controls.Add(_estopBannerLabel, 0, 0);

            _estopDetailLabel = new Label();
            _estopDetailLabel.AutoSize = true;
            _estopDetailLabel.Margin = new Padding(0, 6, 0, 0);
            _estopDetailLabel.ForeColor = Color.FromArgb(150, 172, 196);
            _estopDetailLabel.Text = "等待板端状态帧...";
            estopLayout.Controls.Add(_estopDetailLabel, 0, 1);

            content.Controls.Add(estopCard, 0, 0);

            var scrollPanel = new Panel();
            scrollPanel.Dock = DockStyle.Fill;
            scrollPanel.AutoScroll = true;
            scrollPanel.Padding = new Padding(0, 0, 8, 0);
            content.Controls.Add(scrollPanel, 0, 1);

            var grid = new TableLayoutPanel();
            grid.Dock = DockStyle.Top;
            grid.AutoSize = true;
            grid.AutoSizeMode = AutoSizeMode.GrowAndShrink;
            grid.ColumnCount = 2;
            grid.RowCount = 11;
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 44F));
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 56F));
            for (int row = 0; row < grid.RowCount; row++)
            {
                grid.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            }
            scrollPanel.Controls.Add(grid);

            AddValueRow(grid, 0, "帧序号", "seq");
            AddValueRow(grid, 1, "总急停", "estop");
            AddValueRow(grid, 2, "板端急停", "self_estop");
            AddValueRow(grid, 3, "外部急停", "external_estop");
            AddValueRow(grid, 4, "有效掩码", "valid_mask");
            AddValueRow(grid, 5, "故障掩码", "fault_mask");
            AddValueRow(grid, 6, "越界掩码", "trip_mask");
            AddValueRow(grid, 7, "生效掩码", "active_mask");
            AddValueRow(grid, 8, "Baseline / Tol", "params");
            AddValueRow(grid, 9, "Threshold", "threshold");
            AddValueRow(grid, 10, "解除剩余", "release_remaining");

            return panel;
        }

        private Control BuildLogPanel()
        {
            var panel = CreatePanel();
            panel.Padding = new Padding(14);

            var title = CreateSectionTitle("串口日志");
            panel.Controls.Add(title);

            _logBox = new RichTextBox();
            _logBox.Dock = DockStyle.Fill;
            _logBox.BorderStyle = BorderStyle.None;
            _logBox.BackColor = Color.FromArgb(10, 18, 28);
            _logBox.ForeColor = ForeColor;
            _logBox.Font = new Font("Consolas", 10F, FontStyle.Regular, GraphicsUnit.Point);
            _logBox.ReadOnly = true;
            _logBox.DetectUrls = false;
            _logBox.Margin = new Padding(0, 34, 0, 0);
            panel.Controls.Add(_logBox);
            _logBox.BringToFront();

            return panel;
        }

        private Panel CreatePanel()
        {
            return new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(20, 31, 45),
                BorderStyle = BorderStyle.FixedSingle,
                Margin = new Padding(0, 0, 0, 12)
            };
        }

        private Label CreateSectionTitle(string text)
        {
            return new Label
            {
                Text = text,
                AutoSize = true,
                Font = new Font(Font, FontStyle.Bold),
                ForeColor = Color.FromArgb(81, 229, 255),
                Dock = DockStyle.Top
            };
        }

        private Label CreateCaption(string text)
        {
            return new Label
            {
                Text = text,
                AutoSize = true,
                Margin = new Padding(0, 12, 4, 4),
                ForeColor = Color.FromArgb(150, 172, 196)
            };
        }

        private ComboBox CreateComboBox()
        {
            return new ComboBox
            {
                Dock = DockStyle.Fill,
                DropDownStyle = ComboBoxStyle.DropDownList,
                BackColor = Color.FromArgb(10, 18, 28),
                ForeColor = ForeColor,
                FlatStyle = FlatStyle.Flat
            };
        }

        private Button CreateButton(string text, Action onClick)
        {
            var button = new Button();
            button.Text = text;
            button.Dock = DockStyle.Fill;
            button.FlatStyle = FlatStyle.Flat;
            button.FlatAppearance.BorderColor = Color.FromArgb(70, 101, 132);
            button.BackColor = Color.FromArgb(18, 52, 73);
            button.ForeColor = ForeColor;
            button.Click += delegate { onClick(); };
            return button;
        }

        private NumericUpDown CreateNumericUpDown(int min, int max, int value)
        {
            return new NumericUpDown
            {
                Minimum = min,
                Maximum = max,
                Value = Math.Max(min, Math.Min(max, value)),
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(10, 18, 28),
                ForeColor = ForeColor,
                BorderStyle = BorderStyle.FixedSingle
            };
        }

        private void AddValueRow(TableLayoutPanel grid, int row, string name, string key)
        {
            var left = new Label();
            left.Text = name;
            left.AutoSize = true;
            left.Margin = new Padding(0, 6, 4, 6);
            left.ForeColor = Color.FromArgb(150, 172, 196);
            grid.Controls.Add(left, 0, row);

            var right = new Label();
            right.Text = "--";
            right.AutoSize = true;
            right.Font = new Font("Consolas", 11F, FontStyle.Bold, GraphicsUnit.Point);
            right.Margin = new Padding(0, 6, 4, 6);
            grid.Controls.Add(right, 1, row);
            _valueLabels[key] = right;
        }

        private void ApplySettingsToUi()
        {
            _baudBox.Text = _settings.BaudRate.ToString(CultureInfo.InvariantCulture);
            _autoReconnectBox.Checked = _settings.AutoReconnect;
            if (_releaseHoldInput != null)
            {
                int releaseHoldMs = _settings.ReleaseHoldMs > 0 ? _settings.ReleaseHoldMs : DefaultReleaseHoldMs;
                _releaseHoldInput.Value = ClampToDecimal(_releaseHoldInput, releaseHoldMs);
            }
        }

        private void RefreshPortList()
        {
            string selected = _portBox.SelectedItem as string ?? _settings.PortName ?? string.Empty;
            string[] ports = SerialPort.GetPortNames();
            Array.Sort(ports, StringComparer.OrdinalIgnoreCase);
            _portBox.BeginUpdate();
            _portBox.Items.Clear();
            _portBox.Items.AddRange(ports);
            _portBox.EndUpdate();

            if (!string.IsNullOrWhiteSpace(selected) && _portBox.Items.Contains(selected))
            {
                _portBox.SelectedItem = selected;
            }
            else if (_portBox.Items.Count > 0)
            {
                _portBox.SelectedIndex = 0;
            }
        }

        private void ResetDashboard()
        {
            _hasBoardParameterSync = false;
            _warnedMissingBoardParameters = false;
            _parameterInputsDirty = false;
            _lastBoardBaselineMm = DefaultBaselineMm;
            _lastBoardToleranceMm = DefaultToleranceMm;
            SetValue("seq", "--");
            SetValue("estop", "0");
            SetValue("self_estop", "0");
            SetValue("external_estop", "0");
            SetValue("valid_mask", "0x00");
            SetValue("fault_mask", "0x00");
            SetValue("trip_mask", "0x00");
            SetValue("active_mask", "0x00");
            SetValue("params", "-- / --");
            SetValue("threshold", "--");
            SetValue("release_remaining", "0 ms");
            SetParameterInputs(DefaultBaselineMm, DefaultToleranceMm);
            for (int i = 0; i < _tofLabels.Length; i++)
            {
                _tofLabels[i].Text = "--";
                _tofLabels[i].ForeColor = ForeColor;
            }
            UpdateConnectionLabel();
            UpdateApplyStatus(_serialPort.IsOpen ? "等待板端参数同步" : "未连接板端", Color.Gainsboro);
            UpdateEstopBanner(new H7Frame());
        }

        private void SetValue(string key, string text)
        {
            Label label;
            if (_valueLabels.TryGetValue(key, out label))
            {
                label.Text = text;
                if (key == "estop" || key == "self_estop")
                {
                    label.ForeColor = text == "1" ? Color.FromArgb(255, 107, 107) : Color.FromArgb(103, 240, 169);
                }
                else
                {
                    label.ForeColor = ForeColor;
                }
            }
        }

        private void UpdateEstopBanner(H7Frame frame)
        {
            int estop = frame.Estop;
            int selfEstop = frame.SelfEstop;
            int externalEstop = frame.ExternalEstop;
            bool tripped = estop == 1;
            bool overrideActive = frame.ReleaseRemainingMs > 0;

            if (tripped)
            {
                _estopBannerLabel.Text = "当前状态：急停中";
                _estopBannerLabel.ForeColor = Color.FromArgb(255, 107, 107);
            }
            else if (overrideActive)
            {
                _estopBannerLabel.Text = "当前状态：人工解除生效";
                _estopBannerLabel.ForeColor = Color.LightSkyBlue;
            }
            else
            {
                _estopBannerLabel.Text = "当前状态：运行允许";
                _estopBannerLabel.ForeColor = Color.FromArgb(103, 240, 169);
            }

            string reason;
            if (selfEstop == 1 && externalEstop == 1)
            {
                reason = "板端安全逻辑和外部急停同时触发。";
            }
            else if (overrideActive && frame.FaultMask != 0)
            {
                reason = string.Format(CultureInfo.InvariantCulture, "人工解除剩余 {0} ms；时间到且故障未恢复会再次急停。", frame.ReleaseRemainingMs);
            }
            else if (overrideActive)
            {
                reason = string.Format(CultureInfo.InvariantCulture, "人工解除剩余 {0} ms；当前未检测到板端故障。", frame.ReleaseRemainingMs);
            }
            else if (selfEstop == 1)
            {
                reason = "板端安全逻辑触发急停。";
            }
            else if (externalEstop == 1)
            {
                reason = "外部急停输入触发。";
            }
            else
            {
                reason = "未检测到急停触发条件。";
            }

            _estopDetailLabel.Text = string.Format(
                CultureInfo.InvariantCulture,
                "总急停={0}  板端急停={1}  外部急停={2}  人工解除剩余={3}ms  |  {4}",
                estop,
                selfEstop,
                externalEstop,
                frame.ReleaseRemainingMs,
                reason);
        }

        private void OnRefreshPorts()
        {
            RefreshPortList();
            AppendLog("串口列表已刷新。", Color.LightSkyBlue);
        }

        private void OnConnect()
        {
            TryConnect(false);
        }

        private void OnDisconnect()
        {
            DisconnectPort("已手动断开串口。");
        }

        private void TryConnect(bool silent)
        {
            if (_serialPort.IsOpen)
            {
                return;
            }

            string portName = _portBox.SelectedItem as string;
            if (string.IsNullOrWhiteSpace(portName))
            {
                if (!silent)
                {
                    AppendLog("没有可用串口。", Color.Orange);
                }
                return;
            }

            int baud;
            if (!int.TryParse(_baudBox.Text, NumberStyles.Integer, CultureInfo.InvariantCulture, out baud))
            {
                baud = 115200;
            }

            try
            {
                _serialPort.PortName = portName;
                _serialPort.BaudRate = baud;
                _serialPort.DataBits = 8;
                _serialPort.Parity = Parity.None;
                _serialPort.StopBits = StopBits.One;
                _serialPort.Handshake = Handshake.None;
                _serialPort.Open();

                _settings.PortName = portName;
                _settings.BaudRate = baud;
                _pendingApply = null;
                _hasBoardParameterSync = false;
                _warnedMissingBoardParameters = false;
                _parameterInputsDirty = false;
                _lastBoardBaselineMm = DefaultBaselineMm;
                _lastBoardToleranceMm = DefaultToleranceMm;
                _lastFrameUtc = DateTime.MinValue;
                UpdateConnectionButtons();
                UpdateConnectionLabel();
                UpdateApplyStatus("等待板端参数同步", Color.LightSkyBlue);
                AppendLog(string.Format(CultureInfo.InvariantCulture, "已连接 {0} @ {1}", portName, baud), Color.LightGreen);
            }
            catch (Exception ex)
            {
                UpdateConnectionButtons();
                UpdateConnectionLabel();
                if (!silent)
                {
                    AppendLog("串口连接失败: " + ex.Message, Color.OrangeRed);
                }
            }
        }

        private void DisconnectPort(string reason)
        {
            try
            {
                if (_serialPort.IsOpen)
                {
                    _serialPort.Close();
                }
            }
            catch (Exception ex)
            {
                AppendLog("断开串口异常: " + ex.Message, Color.OrangeRed);
            }

            UpdateConnectionButtons();
            UpdateConnectionLabel();
            ResetDashboard();
            if (!string.IsNullOrWhiteSpace(reason))
            {
                AppendLog(reason, Color.Khaki);
            }
        }
        private void UpdateConnectionButtons()
        {
            bool connected = _serialPort.IsOpen;
            _connectButton.Enabled = !connected;
            _disconnectButton.Enabled = connected;
            _applyParamsButton.Enabled = connected;
            _releaseButton.Enabled = connected;
        }

        private void UpdateConnectionLabel()
        {
            if (_serialPort.IsOpen)
            {
                string age = _lastFrameUtc == DateTime.MinValue ? "等待首帧" : string.Format(CultureInfo.InvariantCulture, "最近帧 {0:0.0}s 前", (DateTime.UtcNow - _lastFrameUtc).TotalSeconds);
                _connectionLabel.Text = "已连接 / " + age;
                _connectionLabel.ForeColor = Color.FromArgb(103, 240, 169);
            }
            else
            {
                _connectionLabel.Text = "未连接";
                _connectionLabel.ForeColor = Color.Gainsboro;
            }
        }

        private void OnApplyParameters()
        {
            if (!_serialPort.IsOpen)
            {
                AppendLog("串口未连接，无法下发参数。", Color.Orange);
                return;
            }

            int baseline = Decimal.ToInt32(_baselineInput.Value);
            int tolerance = Decimal.ToInt32(_toleranceInput.Value);
            int releaseHoldMs = Decimal.ToInt32(_releaseHoldInput.Value);
            string payload = string.Format(CultureInfo.InvariantCulture, "H7CTL,{0},0,{1},{2},{3}", _lineCounter + 1, baseline, tolerance, releaseHoldMs);
            SendTextCommand(WrapFrame(payload));
            _pendingApply = new PendingApply
            {
                BaselineMm = baseline,
                ToleranceMm = tolerance,
                RequestedAtUtc = DateTime.UtcNow
            };
            UpdateApplyStatus("已发送，等待板端确认", Color.Gold);
            if (!_hasBoardParameterSync)
            {
                AppendLog("板端尚未回读 B/T/TH；本次参数会继续下发，但可能无法在工具中确认成功。", Color.Orange);
            }
        }

        private void OnReleaseEstop()
        {
            if (!_serialPort.IsOpen)
            {
                AppendLog("串口未连接，无法发送恢复急停命令。", Color.Orange);
                return;
            }

            int baseline = _hasBoardParameterSync ? _lastBoardBaselineMm : Decimal.ToInt32(_baselineInput.Value);
            int tolerance = _hasBoardParameterSync ? _lastBoardToleranceMm : Decimal.ToInt32(_toleranceInput.Value);
            int releaseHoldMs = Decimal.ToInt32(_releaseHoldInput.Value);
            string payload = string.Format(CultureInfo.InvariantCulture, "H7CTL,{0},1,{1},{2},{3}", _lineCounter + 1, baseline, tolerance, releaseHoldMs);
            SendTextCommand(WrapFrame(payload));
            AppendLog(string.Format(CultureInfo.InvariantCulture, "已发送人工解除命令，持续 {0} ms。时间到且故障仍在时，板端会再次急停。", releaseHoldMs), Color.LightSkyBlue);
        }

        private void SendTextCommand(string command)
        {
            if (!_serialPort.IsOpen)
            {
                AppendLog("串口未连接，无法发送。", Color.Orange);
                return;
            }

            string text = NormalizeOutgoingCommand(command);
            if (string.IsNullOrWhiteSpace(text))
            {
                return;
            }

            try
            {
                _serialPort.Write(text);
                AppendLog(">> " + text.Trim(), Color.LightSkyBlue);
            }
            catch (Exception ex)
            {
                AppendLog("发送失败: " + ex.Message, Color.OrangeRed);
            }
        }

        private static string NormalizeOutgoingCommand(string command)
        {
            string text = (command ?? string.Empty).Trim();
            if (text.Length == 0)
            {
                return string.Empty;
            }
            if (!text.EndsWith("\n", StringComparison.Ordinal))
            {
                text += "\r\n";
            }
            return text;
        }

        private static string WrapFrame(string payload)
        {
            return string.Format(CultureInfo.InvariantCulture, "${0}*{1:X2}", payload, CalcChecksum(payload) & 0xFF);
        }

        private void SerialPortOnErrorReceived(object sender, SerialErrorReceivedEventArgs e)
        {
            BeginInvoke((Action)(() => AppendLog("串口错误: " + e.EventType, Color.OrangeRed)));
        }

        private void SerialPortOnDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                string chunk = _serialPort.ReadExisting();
                if (string.IsNullOrEmpty(chunk))
                {
                    return;
                }

                lock (_sync)
                {
                    _readBuffer.Append(chunk);
                    ExtractCompleteLines();
                }
            }
            catch (Exception ex)
            {
                BeginInvoke((Action)(() => AppendLog("串口读取失败: " + ex.Message, Color.OrangeRed)));
            }
        }

        private void ExtractCompleteLines()
        {
            while (true)
            {
                string bufferText = _readBuffer.ToString();
                int newlineIndex = bufferText.IndexOfAny(new[] { '\r', '\n' });
                if (newlineIndex < 0)
                {
                    break;
                }

                string line = bufferText.Substring(0, newlineIndex).Trim();
                int consumeCount = newlineIndex + 1;
                while (consumeCount < bufferText.Length && (bufferText[consumeCount] == '\r' || bufferText[consumeCount] == '\n'))
                {
                    consumeCount++;
                }

                _readBuffer.Remove(0, consumeCount);
                if (line.Length == 0)
                {
                    continue;
                }

                BeginInvoke((Action<string>)ProcessIncomingLine, line);
            }
        }

        private void ProcessIncomingLine(string line)
        {
            _lastFrameUtc = DateTime.UtcNow;
            UpdateConnectionLabel();
            AppendLog("<< " + line, Color.Gainsboro);

            H7Frame frame;
            if (!TryParseFrame(line, out frame))
            {
                return;
            }

            UpdateFrame(frame);
        }

        private void UpdateFrame(H7Frame frame)
        {
            _lineCounter++;
            if (frame.HasBoardParameters)
            {
                _lastBoardBaselineMm = frame.BaselineMm;
                _lastBoardToleranceMm = frame.ToleranceMm;
                if (!_hasBoardParameterSync)
                {
                    _hasBoardParameterSync = true;
                    _parameterInputsDirty = false;
                    SetParameterInputs(frame.BaselineMm, frame.ToleranceMm);
                    UpdateConnectionButtons();
                    UpdateApplyStatus("已同步板端参数", Color.FromArgb(103, 240, 169));
                    AppendLog(
                        string.Format(
                            CultureInfo.InvariantCulture,
                            "已从板端同步参数: baseline_mm={0}, tolerance_mm={1}, threshold_mm={2}",
                            frame.BaselineMm,
                            frame.ToleranceMm,
                            frame.ThresholdMm),
                        Color.LightGreen);
                }
            }
            else if (!_hasBoardParameterSync && !_warnedMissingBoardParameters)
            {
                _warnedMissingBoardParameters = true;
                UpdateApplyStatus("板端未上报参数", Color.Orange);
                AppendLog("板端状态帧未携带 B/T/TH，当前无法确认参数下发结果；这通常意味着设备仍在运行旧固件。", Color.Orange);
            }
            SetValue("seq", frame.Sequence.ToString(CultureInfo.InvariantCulture));
            SetValue("estop", frame.Estop.ToString(CultureInfo.InvariantCulture));
            SetValue("self_estop", frame.SelfEstop.ToString(CultureInfo.InvariantCulture));
            SetValue("external_estop", frame.ExternalEstop.ToString(CultureInfo.InvariantCulture));
            SetValue("valid_mask", FormatMask(frame.ValidMask));
            SetValue("fault_mask", FormatMask(frame.FaultMask));
            SetValue("trip_mask", FormatMask(frame.TripMask));
            SetValue("active_mask", FormatMask(frame.ActiveMask));
            SetValue("params", frame.HasBoardParameters
                ? string.Format(CultureInfo.InvariantCulture, "{0} / {1}", frame.BaselineMm, frame.ToleranceMm)
                : "-- / --");
            SetValue("threshold", (frame.HasBoardParameters || frame.HasBoardThreshold)
                ? frame.ThresholdMm.ToString(CultureInfo.InvariantCulture)
                : "--");
            SetValue("release_remaining", string.Format(CultureInfo.InvariantCulture, "{0} ms", Math.Max(0, frame.ReleaseRemainingMs)));
            UpdateEstopBanner(frame);

            for (int i = 0; i < 4; i++)
            {
                if (frame.Tof[i] >= 65535)
                {
                    _tofLabels[i].Text = "--";
                    _tofLabels[i].ForeColor = Color.Goldenrod;
                }
                else
                {
                    _tofLabels[i].Text = frame.Tof[i].ToString(CultureInfo.InvariantCulture);
                    bool tripped = ((frame.TripMask >> i) & 0x01) == 1;
                    _tofLabels[i].ForeColor = tripped ? Color.FromArgb(255, 107, 107) : ForeColor;
                }
            }

            if (_pendingApply != null)
            {
                if (!frame.HasBoardParameters)
                {
                    UpdateApplyStatus("已下发，未获板端确认", Color.Orange);
                }
                else if (frame.BaselineMm == _pendingApply.BaselineMm && frame.ToleranceMm == _pendingApply.ToleranceMm)
                {
                    _parameterInputsDirty = false;
                    UpdateApplyStatus("板端已确认", Color.FromArgb(103, 240, 169));
                    _pendingApply = null;
                    SetParameterInputs(frame.BaselineMm, frame.ToleranceMm);
                }
                else
                {
                    UpdateApplyStatus("等待板端回读确认", Color.Gold);
                }
            }
            else
            {
                if (frame.HasBoardParameters && !_parameterInputsDirty)
                {
                    SetParameterInputs(frame.BaselineMm, frame.ToleranceMm);
                }
            }
        }

        private void SetParameterInputs(int baselineMm, int toleranceMm)
        {
            _suppressParameterInputTracking = true;
            try
            {
                _baselineInput.Value = ClampToDecimal(_baselineInput, baselineMm);
                _toleranceInput.Value = ClampToDecimal(_toleranceInput, toleranceMm);
            }
            finally
            {
                _suppressParameterInputTracking = false;
            }
        }

        private void OnParameterInputValueChanged(object sender, EventArgs e)
        {
            if (_suppressParameterInputTracking || _pendingApply != null)
            {
                return;
            }

            int baseline = Decimal.ToInt32(_baselineInput.Value);
            int tolerance = Decimal.ToInt32(_toleranceInput.Value);
            _parameterInputsDirty = (baseline != _lastBoardBaselineMm) || (tolerance != _lastBoardToleranceMm);

            if (_parameterInputsDirty)
            {
                UpdateApplyStatus(_hasBoardParameterSync ? "本地参数已修改，待下发" : "本地参数已修改，待下发（板端未确认）", Color.Gold);
            }
            else
            {
                UpdateApplyStatus(_hasBoardParameterSync ? "已同步板端参数" : "板端参数未确认", _hasBoardParameterSync ? Color.FromArgb(103, 240, 169) : Color.Orange);
            }
        }

        private static decimal ClampToDecimal(NumericUpDown input, int value)
        {
            return Math.Max(input.Minimum, Math.Min(input.Maximum, value));
        }

        private void UpdateApplyStatus(string text, Color color)
        {
            _applyStatusLabel.Text = text;
            _applyStatusLabel.ForeColor = color;
        }

        private void UiTimerOnTick(object sender, EventArgs e)
        {
            UpdateConnectionLabel();

            if (_pendingApply != null && (DateTime.UtcNow - _pendingApply.RequestedAtUtc).TotalSeconds > 3.0)
            {
                UpdateApplyStatus(_hasBoardParameterSync ? "等待超时，请看板端回读" : "已下发，未获板端确认", Color.Orange);
            }

            if (_autoReconnectBox.Checked && !_serialPort.IsOpen)
            {
                TryConnect(true);
            }
        }

        private void AppendLog(string message, Color color)
        {
            string line = string.Format(CultureInfo.InvariantCulture, "[{0:HH:mm:ss}] {1}{2}", DateTime.Now, message, Environment.NewLine);
            _logBox.SelectionStart = _logBox.TextLength;
            _logBox.SelectionLength = 0;
            _logBox.SelectionColor = color;
            _logBox.AppendText(line);
            _logBox.SelectionColor = _logBox.ForeColor;
            _logBox.ScrollToCaret();

            if (_logBox.Lines.Length > 500)
            {
                var lines = _logBox.Lines;
                int keep = Math.Min(400, lines.Length);
                string[] trimmed = new string[keep];
                Array.Copy(lines, lines.Length - keep, trimmed, 0, keep);
                _logBox.Lines = trimmed;
            }
        }

        private void OnFormClosing(object sender, FormClosingEventArgs e)
        {
            _settings.PortName = _portBox.SelectedItem as string ?? _settings.PortName;
            int baud;
            if (int.TryParse(_baudBox.Text, NumberStyles.Integer, CultureInfo.InvariantCulture, out baud))
            {
                _settings.BaudRate = baud;
            }
            _settings.ReleaseHoldMs = Decimal.ToInt32(_releaseHoldInput.Value);
            _settings.AutoReconnect = _autoReconnectBox.Checked;
            SaveSettings(_settings);
            DisconnectPort(string.Empty);
        }

        private AppSettings LoadSettings()
        {
            try
            {
                if (!File.Exists(_settingsPath))
                {
                    return new AppSettings();
                }

                using (var stream = File.OpenRead(_settingsPath))
                {
                    var serializer = new DataContractJsonSerializer(typeof(AppSettings));
                    var result = serializer.ReadObject(stream) as AppSettings;
                    return result ?? new AppSettings();
                }
            }
            catch
            {
                return new AppSettings();
            }
        }

        private void SaveSettings(AppSettings settings)
        {
            try
            {
                using (var stream = File.Create(_settingsPath))
                {
                    var serializer = new DataContractJsonSerializer(typeof(AppSettings));
                    serializer.WriteObject(stream, settings);
                }
            }
            catch
            {
            }
        }

        private static string FormatMask(int value)
        {
            return string.Format(CultureInfo.InvariantCulture, "0x{0:X2}", value & 0xFF);
        }

        private static bool TryParseFrame(string line, out H7Frame frame)
        {
            frame = null;
            string text = (line ?? string.Empty).Trim();
            if (text.Length == 0)
            {
                return false;
            }

            if (text[0] == '{')
            {
                return false;
            }

            string clean = text.StartsWith("$", StringComparison.Ordinal) ? text.Substring(1) : text;
            int starIndex = clean.IndexOf('*');
            string payload = starIndex >= 0 ? clean.Substring(0, starIndex) : clean;
            if (starIndex >= 0)
            {
                int expectedChecksum;
                string checksumText = clean.Substring(starIndex + 1).Trim();
                if (!int.TryParse(checksumText, NumberStyles.HexNumber, CultureInfo.InvariantCulture, out expectedChecksum))
                {
                    return false;
                }
                if ((CalcChecksum(payload) & 0xFF) != (expectedChecksum & 0xFF))
                {
                    return false;
                }
            }
            string[] parts = payload.Split(',');
            if (parts.Length < 9)
            {
                return false;
            }
            if (!string.Equals(parts[0], "H7TOF", StringComparison.OrdinalIgnoreCase) &&
                !string.Equals(parts[0], "TOF", StringComparison.OrdinalIgnoreCase))
            {
                return false;
            }

            frame = new H7Frame();
            frame.Sequence = ParseInt(parts[1], 0);
            frame.Tof[0] = ParseInt(parts[2], 65535);
            frame.Tof[1] = ParseInt(parts[3], 65535);
            frame.Tof[2] = ParseInt(parts[4], 65535);
            frame.Tof[3] = ParseInt(parts[5], 65535);
            frame.Estop = ParseInt(parts[6], 0);
            frame.ValidMask = ParseInt(parts[7], 0);
            frame.FaultMask = ParseInt(parts[8], 0);
            int taggedValue;
            if (TryParseTagged(parts, "TM=", out taggedValue))
            {
                frame.TripMask = taggedValue;
            }
            if (TryParseTagged(parts, "A=", out taggedValue))
            {
                frame.ActiveMask = taggedValue;
            }
            if (TryParseTagged(parts, "SE=", out taggedValue))
            {
                frame.SelfEstop = taggedValue;
            }
            if (TryParseTagged(parts, "EE=", out taggedValue))
            {
                frame.ExternalEstop = taggedValue;
            }

            bool hasBaseline = TryParseTagged(parts, "B=", out taggedValue);
            if (hasBaseline)
            {
                frame.BaselineMm = taggedValue;
            }

            bool hasTolerance = TryParseTagged(parts, "T=", out taggedValue);
            if (hasTolerance)
            {
                frame.ToleranceMm = taggedValue;
            }

            frame.HasBoardParameters = hasBaseline && hasTolerance;
            frame.HasBoardThreshold = TryParseTagged(parts, "TH=", out taggedValue);
            if (frame.HasBoardThreshold)
            {
                frame.ThresholdMm = taggedValue;
            }
            else if (frame.HasBoardParameters)
            {
                frame.ThresholdMm = Math.Min(65534, frame.BaselineMm + frame.ToleranceMm);
            }

            if (TryParseTagged(parts, "RT=", out taggedValue))
            {
                frame.ReleaseRemainingMs = taggedValue;
            }

            return true;
        }

        private static bool TryParseTagged(string[] parts, string prefix, out int value)
        {
            foreach (string part in parts)
            {
                string item = part.Trim();
                if (item.StartsWith(prefix, StringComparison.OrdinalIgnoreCase))
                {
                    value = ParseInt(item.Substring(prefix.Length), 0);
                    return true;
                }
            }
            value = 0;
            return false;
        }

        private static int CalcChecksum(string payload)
        {
            int checksum = 0;
            for (int i = 0; i < payload.Length; i++)
            {
                checksum ^= payload[i];
            }
            return checksum;
        }

        private static int ParseInt(string raw, int fallback)
        {
            if (string.IsNullOrWhiteSpace(raw))
            {
                return fallback;
            }

            raw = raw.Trim();
            int value;
            if (raw.StartsWith("0x", StringComparison.OrdinalIgnoreCase))
            {
                return int.TryParse(raw.Substring(2), NumberStyles.HexNumber, CultureInfo.InvariantCulture, out value) ? value : fallback;
            }

            if (Regex.IsMatch(raw, "^[0-9A-Fa-f]+$") && Regex.IsMatch(raw, "[A-Fa-f]"))
            {
                return int.TryParse(raw, NumberStyles.HexNumber, CultureInfo.InvariantCulture, out value) ? value : fallback;
            }

            return int.TryParse(raw, NumberStyles.Integer, CultureInfo.InvariantCulture, out value) ? value : fallback;
        }
    }

    internal static class Program
    {
        [STAThread]
        private static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new MainForm());
        }
    }
}
