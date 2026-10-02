import 'package:flutter/material.dart';
import 'app_color.dart';

class AppTypography {
  AppTypography._();

  //Headings
  static const TextStyle h1 = TextStyle(
    fontSize: 32,
    fontWeight: FontWeight.bold,
  );
  static const TextStyle h1OnColor = TextStyle(
    fontSize: 32,
    fontWeight: FontWeight.bold,
    color: AppColor.textOnColor,
  );
  static const TextStyle h1Dark = h1OnColor;

  static const TextStyle h2 = TextStyle(
    fontSize: 28,
    fontWeight: FontWeight.bold,
  );
  static const TextStyle h2OnColor = TextStyle(
    fontSize: 28,
    fontWeight: FontWeight.bold,
    color: AppColor.textOnColor,
  );
  static const TextStyle h2Dark = h2OnColor;

  static const TextStyle h3 = TextStyle(
    fontSize: 24,
    fontWeight: FontWeight.w700,
  );
  static const TextStyle h3OnColor = TextStyle(
    fontSize: 24,
    fontWeight: FontWeight.w700,
    color: AppColor.textOnColor,
  );
  static const TextStyle h3Dark = h3OnColor;

  //Titles (20px)
  static const TextStyle title = TextStyle(
    fontSize: 20,
    fontWeight: FontWeight.w600,
  );
  static const TextStyle titleBold = TextStyle(
    fontSize: 20,
    fontWeight: FontWeight.bold,
  );
  static const TextStyle titlePrimary = TextStyle(
    fontSize: 20,
    fontWeight: FontWeight.bold,
    color: AppColor.primaryLight,
  );
  static const TextStyle titleOnColor = TextStyle(
    fontSize: 20,
    fontWeight: FontWeight.w600,
    color: AppColor.textOnColor,
  );
  static const TextStyle titleBoldOnColor = TextStyle(
    fontSize: 20,
    fontWeight: FontWeight.bold,
    color: AppColor.textOnColor,
  );
  static const TextStyle titleDark = titleOnColor;
  static const TextStyle titleBoldDark = titleBoldOnColor;

  //Subtitles (18px)
  static const TextStyle subtitle = TextStyle(
    fontSize: 18,
    fontWeight: FontWeight.w600,
  );
  static const TextStyle subtitleBold = TextStyle(
    fontSize: 18,
    fontWeight: FontWeight.bold,
  );
  static const TextStyle subtitlePrimary = TextStyle(
    fontSize: 18,
    fontWeight: FontWeight.bold,
    color: AppColor.primaryLight,
  );
  static const TextStyle subtitleOnColor = TextStyle(
    fontSize: 18,
    fontWeight: FontWeight.w600,
    color: AppColor.textOnColor,
  );
  static const TextStyle subtitleBoldOnColor = TextStyle(
    fontSize: 18,
    fontWeight: FontWeight.bold,
    color: AppColor.textOnColor,
  );
  static const TextStyle subtitleDark = subtitleOnColor;
  static const TextStyle subtitleBoldDark = subtitleBoldOnColor;

  //Body (16px)
  static const TextStyle body = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.w400,
  );
  static const TextStyle bodyBold = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.bold,
  );
  static const TextStyle bodySub = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.w400,
    color: AppColor.textSecondary,
  );
  static const TextStyle bodySubOnColor = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.w400,
    color: AppColor.textOnColorSecondary,
  );
  static const TextStyle bodyPrimary = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.w600,
    color: AppColor.primaryLight,
  );
  static const TextStyle bodyBoldPrimary = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.bold,
    color: AppColor.primaryLight,
  );
  static const TextStyle bodyError = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.w600,
    color: AppColor.error,
  );
  static const TextStyle bodyBoldError = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.bold,
    color: AppColor.error,
  );
  static const TextStyle bodySuccess = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.w600,
    color: AppColor.success,
  );
  static const TextStyle bodyBoldSuccess = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.bold,
    color: AppColor.success,
  );
  static const TextStyle bodyWarning = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.w600,
    color: AppColor.warning,
  );
  static const TextStyle bodyInfo = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.w600,
    color: AppColor.info,
  );
  static const TextStyle bodyOnColor = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.w400,
    color: AppColor.textOnColor,
  );
  static const TextStyle bodyBoldOnColor = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.bold,
    color: AppColor.textOnColor,
  );
  static const TextStyle bodyDark = bodyOnColor;
  static const TextStyle bodyBoldDark = bodyBoldOnColor;

  //Body Small (14px)
  static const TextStyle bodySmall = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.w400,
  );
  static const TextStyle bodySmallBold = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.w600,
  );
  static const TextStyle bodySmallSub = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.w400,
    color: AppColor.textSecondary,
  );
  static const TextStyle bodySmallSubBold = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.w600,
    color: AppColor.textSecondary,
  );
  static const TextStyle bodySmallHint = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.w400,
    color: AppColor.textHint,
  );
  static const TextStyle bodyHint = bodySmallHint;

  static const TextStyle bodySmallPrimary = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.w600,
    color: AppColor.primaryLight,
  );
  static const TextStyle bodySmallBoldPrimary = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.bold,
    color: AppColor.primaryLight,
  );
  static const TextStyle bodySmallError = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.w600,
    color: AppColor.error,
  );
  static const TextStyle bodySmallBoldError = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.bold,
    color: AppColor.error,
  );
  static const TextStyle bodySmallSuccess = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.w600,
    color: AppColor.success,
  );
  static const TextStyle bodySmallBoldSuccess = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.bold,
    color: AppColor.success,
  );
  static const TextStyle bodySmallWarning = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.w600,
    color: AppColor.warning,
  );
  static const TextStyle bodySmallInfo = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.w600,
    color: AppColor.info,
  );
  static const TextStyle bodySmallBoldInfo = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.bold,
    color: AppColor.info,
  );
  static const TextStyle bodySmallOnColor = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.w400,
    color: AppColor.textOnColor,
  );
  static const TextStyle bodySmallBoldOnColor = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.w600,
    color: AppColor.textOnColor,
  );
  static const TextStyle bodySmallSubOnColor = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.w400,
    color: AppColor.textOnColorSecondary,
  );
  static const TextStyle bodySmallDark = bodySmallOnColor;
  static const TextStyle bodySmallBoldDark = bodySmallBoldOnColor;

  //Caption (12px)
  static const TextStyle caption = TextStyle(
    fontSize: 12,
    fontWeight: FontWeight.w400,
  );
  static const TextStyle captionBold = TextStyle(
    fontSize: 12,
    fontWeight: FontWeight.w600,
  );
  static const TextStyle captionSub = TextStyle(
    fontSize: 12,
    fontWeight: FontWeight.w400,
    color: AppColor.textSecondary,
  );
  static const TextStyle captionHint = TextStyle(
    fontSize: 12,
    fontWeight: FontWeight.w400,
    color: AppColor.textHint,
  );
  static const TextStyle captionPrimary = TextStyle(
    fontSize: 12,
    fontWeight: FontWeight.w600,
    color: AppColor.primaryLight,
  );
  static const TextStyle captionBoldPrimary = TextStyle(
    fontSize: 12,
    fontWeight: FontWeight.bold,
    color: AppColor.primaryLight,
  );
  static const TextStyle captionError = TextStyle(
    fontSize: 12,
    fontWeight: FontWeight.w600,
    color: AppColor.error,
  );
  static const TextStyle captionBoldError = TextStyle(
    fontSize: 12,
    fontWeight: FontWeight.bold,
    color: AppColor.error,
  );
  static const TextStyle captionSuccess = TextStyle(
    fontSize: 12,
    fontWeight: FontWeight.w600,
    color: AppColor.success,
  );
  static const TextStyle captionWarning = TextStyle(
    fontSize: 12,
    fontWeight: FontWeight.w600,
    color: AppColor.warning,
  );
  static const TextStyle captionInfo = TextStyle(
    fontSize: 12,
    fontWeight: FontWeight.w600,
    color: AppColor.info,
  );
  static const TextStyle captionOnColor = TextStyle(
    fontSize: 12,
    fontWeight: FontWeight.w400,
    color: AppColor.textOnColor,
  );
  static const TextStyle captionBoldOnColor = TextStyle(
    fontSize: 12,
    fontWeight: FontWeight.w600,
    color: AppColor.textOnColor,
  );
  static const TextStyle captionSubOnColor = TextStyle(
    fontSize: 12,
    fontWeight: FontWeight.w400,
    color: AppColor.textOnColorSecondary,
  );
  static const TextStyle captionDark = captionOnColor;
  static const TextStyle captionBoldDark = captionBoldOnColor;

  //Tiny (10px)
  static const TextStyle tiny = TextStyle(
    fontSize: 10,
    fontWeight: FontWeight.w400,
  );
  static const TextStyle tinyBold = TextStyle(
    fontSize: 10,
    fontWeight: FontWeight.bold,
  );
  static const TextStyle tinySub = TextStyle(
    fontSize: 10,
    fontWeight: FontWeight.w400,
    color: AppColor.textSecondary,
  );
  static const TextStyle tinyPrimary = TextStyle(
    fontSize: 10,
    fontWeight: FontWeight.w600,
    color: AppColor.primaryLight,
  );
  static const TextStyle tinyBoldPrimary = TextStyle(
    fontSize: 10,
    fontWeight: FontWeight.bold,
    color: AppColor.primaryLight,
  );
  static const TextStyle tinyError = TextStyle(
    fontSize: 10,
    fontWeight: FontWeight.w600,
    color: AppColor.error,
  );
  static const TextStyle tinyBoldError = TextStyle(
    fontSize: 10,
    fontWeight: FontWeight.bold,
    color: AppColor.error,
  );
  static const TextStyle tinySuccess = TextStyle(
    fontSize: 10,
    fontWeight: FontWeight.w600,
    color: AppColor.success,
  );
  static const TextStyle tinyOnColor = TextStyle(
    fontSize: 10,
    fontWeight: FontWeight.w400,
    color: AppColor.textOnColor,
  );
  static const TextStyle tinyBoldOnColor = TextStyle(
    fontSize: 10,
    fontWeight: FontWeight.bold,
    color: AppColor.textOnColor,
  );
  static const TextStyle tinySubOnColor = TextStyle(
    fontSize: 10,
    fontWeight: FontWeight.w400,
    color: AppColor.textOnColorSecondary,
  );
  static const TextStyle tinyDark = tinyOnColor;
  static const TextStyle tinyBoldDark = tinyBoldOnColor;

  //Button Styles
  static const TextStyle button = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.w600,
    color: AppColor.textOnColor,
  );
  static const TextStyle buttonBold = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.bold,
    color: AppColor.textOnColor,
  );
  static const TextStyle buttonSmall = TextStyle(
    fontSize: 14,
    fontWeight: FontWeight.w600,
    color: AppColor.textOnColor,
  );
  static const TextStyle buttonOnColor = button;
  static const TextStyle buttonBoldOnColor = buttonBold;
  static const TextStyle buttonSmallOnColor = buttonSmall;
  static const TextStyle buttonPrimary = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.w600,
    color: AppColor.primaryLight,
  );
  static const TextStyle buttonOutline = TextStyle(
    fontSize: 16,
    fontWeight: FontWeight.w600,
  );

  //Logo Styles
  static const TextStyle logo = TextStyle(
    fontSize: 36,
    fontWeight: FontWeight.bold,
    letterSpacing: -1.0,
  );
  static const TextStyle logoLight = TextStyle(
    fontSize: 36,
    fontWeight: FontWeight.bold,
    letterSpacing: -1.0,
    color: AppColor.primaryLight,
  );
  static const TextStyle logoOnColor = TextStyle(
    fontSize: 36,
    fontWeight: FontWeight.bold,
    letterSpacing: -1.0,
    color: AppColor.textOnColor,
  );
}