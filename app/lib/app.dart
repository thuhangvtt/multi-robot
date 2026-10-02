import 'package:flutter/material.dart';

import 'features/robot_control/presentation/robot_control_screen.dart';
import 'widget/app_color.dart';
import 'widget/app_typography.dart';

class RobotControlApp extends StatelessWidget {
  const RobotControlApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Robot Control',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: AppColor.primaryLight),
        scaffoldBackgroundColor: AppColor.bgDefault,
        textTheme: const TextTheme(
          bodyLarge: AppTypography.body,
          bodyMedium: AppTypography.body,
          bodySmall: AppTypography.bodySmall,
          titleMedium: AppTypography.title,
        ),
        useMaterial3: true,
      ),
      home: const RobotControlScreen(),
    );
  }
}
