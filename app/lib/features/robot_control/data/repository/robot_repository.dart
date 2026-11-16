import 'dart:convert';
import 'package:http/http.dart' as http;

class RobotRepository {
  RobotRepository(this.host);
  final String host;
  final http.Client _client = http.Client();

  Future<void> sendCommand(Map<String, String> parameters) async {
    final response = await _client.post(Uri.http(host, '/cmd'), body: {
      'l': '90', 'r': '90', 'action': '0', ...parameters,
    }).timeout(const Duration(milliseconds: 500));
    if (response.statusCode != 200) {
      throw Exception('HTTP ${response.statusCode}: ${response.body}');
    }
  }

  Future<void> emergencyStop() async {
    final response = await _client.get(Uri.http(host, '/stop'))
        .timeout(const Duration(milliseconds: 500));
    if (response.statusCode != 200) throw Exception('STOP failed: HTTP ${response.statusCode}');
  }

  Future<Map<String, dynamic>> fetchStatus() async {
    final response = await _client.get(Uri.http(host, '/status'))
        .timeout(const Duration(seconds: 1));
    if (response.statusCode != 200) throw Exception('HTTP ${response.statusCode}');
    return jsonDecode(response.body) as Map<String, dynamic>;
  }

  void close() => _client.close();
}
