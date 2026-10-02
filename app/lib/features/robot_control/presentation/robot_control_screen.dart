import 'package:flutter/material.dart';

import '../domain/robot_controller.dart';
import '../../../widget/app_color.dart';
import '../../../widget/app_typography.dart';

class RobotControlScreen extends StatefulWidget {
  const RobotControlScreen({super.key});

  @override
  State<RobotControlScreen> createState() => _RobotControlScreenState();
}

class _RobotControlScreenState extends State<RobotControlScreen> {
  final _hostController = TextEditingController(text: '192.168.1.100');
  late final RobotController _controller;

  @override
  void initState() {
    super.initState();
    _controller = RobotController();
    _controller.addListener(_refresh);
    _controller.initialize();
  }

  @override
  void dispose() {
    _controller.removeListener(_refresh);
    _controller.dispose();
    _hostController.dispose();
    super.dispose();
  }

  void _refresh() {
    if (mounted) setState(() {});
  }

  Widget _directionButton({required IconData icon, required String label, required VoidCallback onStart}) {
    return GestureDetector(
      onTapDown: (_) => onStart(),
      onTapUp: (_) => _controller.stopDrive(),
      onTapCancel: _controller.stopDrive,
      child: Container(
        width: 88,
        height: 72,
        decoration: BoxDecoration(color: AppColor.primarySurface, borderRadius: BorderRadius.circular(18), border: Border.all(color: AppColor.primaryLight)),
        child: Column(mainAxisAlignment: MainAxisAlignment.center, children: [Icon(icon, color: AppColor.primaryDark, size: 30), Text(label, style: AppTypography.tinyBold)]),
      ),
    );
  }

  Widget _panel({required String title, required IconData icon, required Widget child}) {
    return Container(
      padding: const EdgeInsets.all(18),
      decoration: BoxDecoration(color: AppColor.bgCard, borderRadius: BorderRadius.circular(22), boxShadow: const [BoxShadow(color: AppColor.overlayLight, blurRadius: 18, offset: Offset(0, 6))]),
      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [Row(children: [Icon(icon, color: AppColor.primaryLight), const SizedBox(width: 8), Text(title, style: AppTypography.subtitleBold)]), const SizedBox(height: 16), child]),
    );
  }

  @override
  Widget build(BuildContext context) {
    final leftMotor = _controller.status['left']?.toString() ?? '--';
    final rightMotor = _controller.status['right']?.toString() ?? '--';
    final yaw = _controller.status['yaw']?.toString() ?? '--';
    return Scaffold(
      appBar: AppBar(title: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [Text('ROBOT CONTROL', style: AppTypography.titleBold), Text('Điều khiển xe', style: AppTypography.captionSub)]), actions: [IconButton(onPressed: _controller.refreshStatus, icon: const Icon(Icons.refresh), tooltip: 'Kiểm tra kết nối')]),
      body: SafeArea(child: ListView(padding: const EdgeInsets.fromLTRB(16, 8, 16, 28), children: [
        _connectionPanel(),
        const SizedBox(height: 14),
        _panel(title: 'Điều hướng', icon: Icons.gamepad_outlined, child: Column(children: [
          _directionButton(icon: Icons.keyboard_arrow_up, label: 'TIẾN', onStart: () => _controller.startDrive(1, 1)),
          const SizedBox(height: 8),
          Row(mainAxisAlignment: MainAxisAlignment.center, children: [
            _directionButton(icon: Icons.keyboard_arrow_left, label: 'TRÁI', onStart: () => _controller.startDrive(-1, 1)),
            const SizedBox(width: 12),
            GestureDetector(onTap: _controller.stopDrive, child: Container(width: 88, height: 72, decoration: BoxDecoration(color: AppColor.errorDark, borderRadius: BorderRadius.circular(18)), child: Column(mainAxisAlignment: MainAxisAlignment.center, children: [const Icon(Icons.stop, color: AppColor.textOnColor, size: 30), Text('DỪNG', style: AppTypography.tinyBoldOnColor)]))),
            const SizedBox(width: 12),
            _directionButton(icon: Icons.keyboard_arrow_right, label: 'PHẢI', onStart: () => _controller.startDrive(1, -1)),
          ]),
          const SizedBox(height: 8),
          _directionButton(icon: Icons.keyboard_arrow_down, label: 'LÙI', onStart: () => _controller.startDrive(-1, -1)),
          const SizedBox(height: 12),
          Row(children: [const Text('Tốc độ', style: AppTypography.bodyBold), Expanded(child: Slider(value: _controller.speed.toDouble(), min: 50, max: 255, divisions: 41, label: '${_controller.speed}', onChanged: (value) => _controller.updateSpeed(value.round()))), Text('${_controller.speed}', style: AppTypography.bodyBold)]),
        ])),
        const SizedBox(height: 14),
        _panel(title: 'Cơ cấu servo', icon: Icons.tune, child: Column(children: [_servoSlider('Servo A', _controller.servoA, _controller.updateServoA), _servoSlider('Servo B', _controller.servoB, _controller.updateServoB)])),
        const SizedBox(height: 14),
        _panel(title: 'Hỗ trợ lái', icon: Icons.explore_outlined, child: Column(children: [
          Material(color: Colors.transparent, child: SwitchListTile(contentPadding: EdgeInsets.zero, title: const Text('Giữ hướng bằng gyro'), subtitle: const Text('Robot tự cân bằng khi chạy thẳng'), value: _controller.straight, onChanged: _controller.setStraight)),
          const Divider(),
          Material(color: Colors.transparent, child: ListTile(contentPadding: EdgeInsets.zero, leading: const Icon(Icons.explore, color: AppColor.primaryLight), title: const Text('Đặt lại góc yaw'), subtitle: Text('Yaw hiện tại: $yaw°'), trailing: IconButton(onPressed: () => _controller.send({'dtype': 'zeroYaw'}, success: 'Đã đặt lại yaw'), icon: const Icon(Icons.restart_alt), tooltip: 'Đặt lại yaw'))),
        ])),
        const SizedBox(height: 14),
        Text('Motor trái: $leftMotor    Motor phải: $rightMotor', textAlign: TextAlign.center, style: AppTypography.captionSub),
      ])),
    );
  }

  Widget _connectionPanel() {
    return Container(
      padding: const EdgeInsets.all(18),
      decoration: BoxDecoration(color: AppColor.primaryDark, borderRadius: BorderRadius.circular(22)),
      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
        Row(children: [Container(width: 10, height: 10, decoration: BoxDecoration(color: _controller.online ? AppColor.success : AppColor.warning, shape: BoxShape.circle)), const SizedBox(width: 8), Text(_controller.online ? 'ĐANG KẾT NỐI' : 'CHƯA KẾT NỐI', style: AppTypography.captionBoldOnColor.copyWith(letterSpacing: 1.2)), const Spacer(), Expanded(child: Text(_controller.message, maxLines: 2, overflow: TextOverflow.ellipsis, textAlign: TextAlign.end, style: AppTypography.captionSubOnColor))]),
        const SizedBox(height: 14),
        TextField(controller: _hostController, keyboardType: TextInputType.url, style: AppTypography.bodyOnColor, onChanged: _controller.setHost, decoration: InputDecoration(labelText: 'Địa chỉ ESP32', labelStyle: AppTypography.captionSubOnColor, hintText: '192.168.1.100', hintStyle: AppTypography.captionSubOnColor, prefixIcon: const Icon(Icons.wifi, color: AppColor.iconOnColor), suffixIcon: IconButton(onPressed: _controller.refreshStatus, icon: const Icon(Icons.link, color: AppColor.iconOnColor), tooltip: 'Kết nối'), enabledBorder: const UnderlineInputBorder(borderSide: BorderSide(color: AppColor.textOnColorSecondary)), focusedBorder: const UnderlineInputBorder(borderSide: BorderSide(color: AppColor.textOnColor))),),
      ]),
    );
  }

  Widget _servoSlider(String label, int value, ValueChanged<int> onChanged) {
    return Row(children: [SizedBox(width: 62, child: Text(label, style: AppTypography.bodySmall)), Expanded(child: Slider(value: value.toDouble(), min: 0, max: 180, divisions: 180, label: '$value°', onChanged: (next) => onChanged(next.round()))), Text('$value°', style: AppTypography.bodyBold)]);
  }
}
