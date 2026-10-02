import 'dart:convert';

import 'package:http/http.dart' as http;

class RobotRepository {
  RobotRepository(this.host);

  final String host;

  Uri _control(Map<String, String> parameters) => Uri.http(host, '/control', parameters);

  Future<void> sendCommand(Map<String, String> parameters) async {
    final response = await http.get(_control(parameters)).timeout(const Duration(milliseconds: 700));
    if (response.statusCode >= 400) throw Exception('HTTP ${response.statusCode}');
  }

  Future<Map<String, dynamic>> fetchStatus() async {
    final response = await http.get(Uri.http(host, '/status')).timeout(const Duration(seconds: 2));
    if (response.statusCode >= 400) throw Exception('HTTP ${response.statusCode}');
    return jsonDecode(response.body) as Map<String, dynamic>;
  }
}
