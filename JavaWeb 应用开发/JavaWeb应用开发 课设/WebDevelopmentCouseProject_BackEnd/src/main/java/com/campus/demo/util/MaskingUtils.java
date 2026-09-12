package com.campus.demo.util;

public final class MaskingUtils {

    private MaskingUtils() {
    }

    public static String maskMobile(String mobile) {
        if (mobile == null || mobile.length() < 7) {
            return mobile;
        }
        return mobile.substring(0, 3) + "****" + mobile.substring(mobile.length() - 4);
    }

    public static String maskName(String name) {
        if (name == null || name.isBlank()) {
            return name;
        }
        if (name.length() == 1) {
            return name;
        }
        if (name.length() == 2) {
            return name.charAt(0) + "*";
        }
        return name.charAt(0) + "**";
    }
}
