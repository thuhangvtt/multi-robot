import 'dart:async';

import '../data/repository/robot_repository.dart';

class RobotController {
  RobotController({this.host = '192.168.1.100'});

  String host;
  RobotRepository get _repository => RobotRepository(host);
  Timer? _driveTimer;
  Timer? _statusTimer;
  Future<void> _commandQueue = Future<void>.value();
  final List<void Function()> _listeners = [];

  bool online = false;
  bool straight = false;
  int speed = 150;
  int servoA = 90;
  int servoB = 90;
  String message = 'Chưa kết nối';
  Map<String, dynamic> status = const {};

  void addListener(void Function() listener) => _listeners.add(listener);
  void removeListener(void Function() listener) => _listeners.remove(listener);
  void _notify() {
    for (final listener in List.of(_listeners)) {
      listener();
    }
  }

  void setHost(String host) {
    this.host = host.trim();
  }

  void initialize() {
    _statusTimer = Timer.periodic(const Duration(seconds: 3), (_) => refreshStatus());
  }

  Future<void> send(Map<String, String> parameters, {String? success}) {
    final operation = _commandQueue.then((_) async {
      try {
        await _repository.sendCommand(parameters);
        online = true;
        message = success ?? 'Đã gửi lệnh';
      } catch (error) {
        message = 'Lệnh thất bại: $error';
      }
      _notify();
    });

    _commandQueue = operation.catchError((_) {});
    return operation;
  }

  Future<void> refreshStatus() async {
    try {
      status = await _repository.fetchStatus();
      online = true;
      message = 'Robot đang sẵn sàng';
    } catch (_) {
      online = false;
    }
    _notify();
  }

  void startDrive(int left, int right) {
    _driveTimer?.cancel();

    void sendDrive() {
      send({
        'dtype': 'speed',
        'left': '${left * speed}',
        'right': '${right * speed}',
      });
    }

    sendDrive();
    _driveTimer = Timer.periodic(const Duration(milliseconds: 300), (_) => sendDrive());
  }

  void stopDrive() {
    _driveTimer?.cancel();
    _driveTimer = null;
    send({'dtype': 'stop'}, success: 'Đã dừng robot');
  }

  void updateSpeed(int value) {
    speed = value;
    _notify();
  }

  void updateServoA(int value) {
    servoA = value;
    _notify();
    send({'dtype': 'servo', 'a': '$servoA', 'b': '$servoB'}, success: 'Đã cập nhật servo');
  }

  void updateServoB(int value) {
    servoB = value;
    _notify();
    send({'dtype': 'servo', 'a': '$servoA', 'b': '$servoB'}, success: 'Đã cập nhật servo');
  }

  void setStraight(bool value) {
    straight = value;
    _notify();
    send({'dtype': 'straight', 'enabled': value ? '1' : '0'});
  }

  void dispose() {
    _driveTimer?.cancel();
    _statusTimer?.cancel();
    _listeners.clear();
  }
}
