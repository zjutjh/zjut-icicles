package com.campus.demo.exception;

import com.campus.demo.common.Result;
import com.fasterxml.jackson.databind.exc.InvalidFormatException;
import org.springframework.http.converter.HttpMessageNotReadableException;
import org.springframework.validation.FieldError;
import org.springframework.web.bind.MethodArgumentNotValidException;
import org.springframework.web.bind.annotation.ExceptionHandler;
import org.springframework.web.bind.annotation.RestControllerAdvice;
import org.springframework.web.method.annotation.MethodArgumentTypeMismatchException;

import java.util.LinkedHashMap;
import java.util.Map;

@RestControllerAdvice
public class GlobalExceptionHandler {

    @ExceptionHandler(MethodArgumentNotValidException.class)
    public Result<Map<String, String>> handleValidException(MethodArgumentNotValidException ex) {
        Map<String, String> errors = new LinkedHashMap<>();
        for (FieldError fieldError : ex.getBindingResult().getFieldErrors()) {
            errors.putIfAbsent(fieldError.getField(), fieldError.getDefaultMessage());
        }
        return new Result<>(400, "参数校验失败", errors);
    }

    @ExceptionHandler(MethodArgumentTypeMismatchException.class)
    public Result<String> handleTypeMismatchException(MethodArgumentTypeMismatchException ex) {
        String paramName = ex.getName();
        Object value = ex.getValue();
        Class<?> requiredType = ex.getRequiredType();
        String typeName = requiredType == null ? "目标类型" : requiredType.getSimpleName();
        String detail = "参数 " + paramName + " 的值 " + value + " 无法转换为 " + typeName;
        return new Result<>(400, "参数类型错误", detail);
    }

    @ExceptionHandler(HttpMessageNotReadableException.class)
    public Result<String> handleHttpMessageNotReadableException(HttpMessageNotReadableException ex) {
        Throwable cause = ex.getCause();
        if (cause instanceof InvalidFormatException invalidFormatException && !invalidFormatException.getPath().isEmpty()) {
            String fieldName = invalidFormatException.getPath().get(0).getFieldName();
            Class<?> targetType = invalidFormatException.getTargetType();
            if (targetType != null && targetType.isEnum()) {
                String detail = "参数 " + fieldName + " 的值 " + invalidFormatException.getValue() + " 不在允许范围内";
                return new Result<>(400, "参数校验失败", detail);
            }
        }
        return new Result<>(400, "参数格式错误", "请求体格式不正确");
    }

    @ExceptionHandler(IllegalArgumentException.class)
    public Result<String> handleIllegalArgumentException(IllegalArgumentException ex) {
        return Result.fail(ex.getMessage());
    }

    @ExceptionHandler(Exception.class)
    public Result<String> handleUnexpectedException(Exception ex) {
        return new Result<>(500, "服务器内部错误", ex.getMessage());
    }
}
