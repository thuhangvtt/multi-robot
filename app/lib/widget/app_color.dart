import 'package:flutter/material.dart';

class AppColor {
  AppColor._();

  // Brand Colors
  static const Color primaryDark    = Color(0xFF002F21); // Xanh lá đậm
  static const Color primaryLight   = Color(0xFF26994F); // Xanh lá chính
  static const Color primarySurface = Color(0xFFE8F5E9); // Nền xanh nhạt

  static const Color error          = Color(0xFFE53935); // Đỏ chính
  static const Color errorDark      = Color(0xFFC62828); // Đỏ đậm (dùng gradient)
  static const Color errorSurface   = Color(0xFFFFEBEE); // Nền đỏ nhạt

  static const Color success        = Color(0xFF26994F); // Xanh lá
  static const Color successDark    = Color(0xFF1B7A3E); // Xanh lá đậm
  static const Color successSurface = Color(0xFFE8F5E9); // Nền xanh nhạt

  static const Color warning        = Color(0xFFFFC107); // Vàng cam
  static const Color warningSurface = Color(0xFFFFF8E1); // Nền vàng nhạt

  static const Color info           = Color(0xFF1565C0); // Xanh dương
  static const Color infoDark       = Color(0xFF0D47A1); // Xanh dương đậm (dùng gradient)
  static const Color infoSurface    = Color(0xFFE3F2FD); // Nền xanh dương nhạt

  // Accent Colors (Icon / Category / Tag)
  static const Color accentPurple   = Color(0xFF7C4DFF); // Tím
  static const Color accentCyan     = Color(0xFF00ACC1); // Cyan / Xanh ngọc
  static const Color accentOrange   = Color(0xFFFF7043); // Cam

  /// Chữ chính trên nền sáng
  static const Color textPrimary          = Color(0xFF1A1A2E);
  /// Chữ phụ / mô tả trên nền sáng
  static const Color textSecondary        = Color(0xFF757575);
  /// Gợi ý / placeholder trên nền sáng
  static const Color textHint             = Color(0xFF9E9E9E);
  /// Chữ trên nền màu (xanh, gradient, tối)
  static const Color textOnColor          = Color(0xFFFFFFFF);
  /// Chữ phụ trên nền tối / gradient
  static const Color textOnColorSecondary = Color(0xB3FFFFFF);

  //Background & Surface Colors (Light)
  static const Color bgDefault  = Color(0xFFF0EFF8); // Nền màn hình
  static const Color bgCard     = Color(0xFFFFFFFF); // Thẻ card
  static const Color bgElevated = Color(0xFFF5F5F5); // Surface nổi
  static const Color bgField    = Color(0xFFF0F0F0); // Input field

  // Background & Surface Colors (Dark)
  static const Color bgDark         = Color(0xFF121212); // Nền màn hình dark
  static const Color bgCardDark     = Color(0xFF1E1E1E); // Thẻ card dark
  static const Color bgElevatedDark = Color(0xFF2C2C2C); // Surface nổi dark
  static const Color bgFieldDark    = Color(0xFF2C2C2C); // Input field dark

  // Border & Divider
  static const Color borderDefault  = Color(0xFFE0E0E0);
  static const Color borderDark     = Color(0x3DFFFFFF); // Trắng 24%
  static const Color dividerDefault = Color(0xFFEEEEEE);
  static const Color dividerDark    = Color(0x1FFFFFFF); // Trắng 12%

  // Basic & Overlay
  static const Color white        = Color(0xFFFFFFFF);
  static const Color black        = Color(0xFF000000);
  static const Color transparent  = Color(0x00000000);
  static const Color overlayLight = Color(0x1A000000); // Đen 10%
  static const Color overlayDark  = Color(0x1AFFFFFF); // Trắng 10%

  // Icon Colors
  static const Color iconPrimary   = Color(0xFF1A1A2E); // Icon chính
  static const Color iconSecondary = Color(0xFF9E9E9E); // Icon phụ
  static const Color iconOnColor   = Color(0xFFFFFFFF); // Icon trên nền màu
}