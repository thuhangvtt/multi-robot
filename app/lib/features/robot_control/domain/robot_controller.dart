import 'dart:async';
import '../data/repository/robot_repository.dart';

class RobotController {
  RobotController({this.host = '192.168.100.101'}) {
    _repository = RobotRepository(host);
  }
  String host;
  late RobotRepository _repository;
  Timer? _driveTimer;
  Timer? _statusTimer;
  bool _sending = false;
  bool _disposed = false;
  int _generation = 0;
  final List<void Function()> _listeners = [];
  bool online = false;
  bool straight = true;
  int speed = 5; 
  int servoA = 0;
  int servoB = 0;
  String message = 'Chưa kết nối';
  Map<String, dynamic> status = const {};

  void addListener(void Function() listener) => _listeners.add(listener);
  void removeListener(void Function() listener) => _listeners.remove(listener);
  void _notify() {
    if (!_disposed) {
      for (final listener in List.of(_listeners)) { listener(); }
    }
  }

  void setHost(String value) {
    ++_generation;
    _driveTimer?.cancel();
    final previous = _repository;
    if (_driveTimer != null) {
      unawaited(previous.emergencyStop().catchError((Object _) {}).whenComplete(previous.close));
    } else {
      previous.close();
    }
    _driveTimer = null;
    host = value.trim();
    _repository = RobotRepository(host);
    online = false;
  }

  void initialize() {
    _statusTimer = Timer.periodic(const Duration(seconds: 3), (_) => refreshStatus());
  }

  Future<void> send(Map<String, String> parameters, {String? success}) async {
    if (_sending || _disposed) return;
    _sending = true;
    try {
      await _repository.sendCommand(parameters);
      online = true;
      message = success ?? 'Đã gửi lệnh';
    } catch (error) {
      _driveTimer?.cancel();
      message = 'Lệnh thất bại: $error';
      online = false;
      try { await _repository.emergencyStop(); } catch (_) { /* watchdog fallback */ }
    } finally {
      _sending = false;
      _notify();
    }
  }

  Future<void> refreshStatus() async {
    try {
      status = await _repository.fetchStatus();
      online = status.containsKey('robotId');
      message = 'Robot online (HTTP)';
    } catch (_) { online = false; }
    _notify();
  }

  void startDrive(int left, int right) {
    _driveTimer?.cancel();
    final generation = ++_generation;
    void drive() {
      if (generation == _generation) {
        send({'l': '${90 + left * speed}', 'r': '${90 + right * speed}'});
      }
    }
    drive();
    _driveTimer = Timer.periodic(const Duration(milliseconds: 150), (_) => drive());
  }

  void stopDrive() {
    ++_generation;
    _driveTimer?.cancel();
    _driveTimer = null;
    unawaited(_stopAfterPendingSend());
  }

  Future<void> _stopAfterPendingSend() async {
    while (_sending && !_disposed) { await Future<void>.delayed(const Duration(milliseconds: 10)); }
    await _neutral();
  }

  Future<void> _neutral() async {
    try { await _repository.sendCommand({'l': '90', 'r': '90'}); }
    catch (_) { try { await _repository.emergencyStop(); } catch (_) { /* watchdog fallback */ } }
  }

  Future<void> emergencyStop() async {
    ++_generation;
    _driveTimer?.cancel();
    try {
      while (_sending) { await Future<void>.delayed(const Duration(milliseconds: 10)); }
      await _repository.emergencyStop();
      message = 'Đã dừng robot';
    } catch (error) { message = 'Chưa xác nhận STOP: $error'; }
    _notify();
  }

  void updateSpeed(int value) { speed = value.clamp(1, 15); _notify(); }
  void updateServoA(int value) {
    servoA = value;
    send({'servo_a': '$servoA', 'servo_b': '$servoB'});
    _notify();
  }
  void updateServoB(int value) {
    servoB = value;
    send({'servo_a': '$servoA', 'servo_b': '$servoB'});
    _notify();
  }
  void setStraight(bool value) {
    straight = value;
    send({'stabilizer': value ? '1' : '0'});
    _notify();
  }
  void dispose() {
    stopDrive();
    _disposed = true;
    _statusTimer?.cancel();
    _listeners.clear();
    Timer(const Duration(seconds: 1), _repository.close);
  }
}
