package com.campus.demo.controller;

import com.campus.demo.common.Result;
import com.campus.demo.dto.ChangePasswordRequest;
import com.campus.demo.dto.LoginRequest;
import com.campus.demo.dto.RegisterUserRequest;
import com.campus.demo.entity.CurrentUser;
import com.campus.demo.entity.LoginResponseData;
import com.campus.demo.entity.SysUser;
import com.campus.demo.service.DemoStoreService;
import jakarta.validation.Valid;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/api/auth")
public class AuthController {

    private final DemoStoreService demoStoreService;

    public AuthController(DemoStoreService demoStoreService) {
        this.demoStoreService = demoStoreService;
    }

    @PostMapping("/login")
    public Result<LoginResponseData> login(@Valid @RequestBody LoginRequest request) {
        return Result.ok(demoStoreService.login(request));
    }

    @PostMapping("/register")
    public Result<SysUser> register(@Valid @RequestBody RegisterUserRequest request) {
        return Result.ok(demoStoreService.registerUser(request));
    }

    @PostMapping("/logout")
    public Result<Boolean> logout(@RequestHeader(value = "Authorization", required = false) String authorization) {
        return Result.ok(demoStoreService.logout(authorization));
    }

    @GetMapping("/me")
    public Result<CurrentUser> me(@RequestHeader(value = "Authorization", required = false) String authorization) {
        return Result.ok(demoStoreService.currentUser(authorization));
    }

    @PostMapping("/change-password")
    public Result<Boolean> changePassword(
            @RequestHeader(value = "Authorization", required = false) String authorization,
            @Valid @RequestBody ChangePasswordRequest request) {
        return Result.ok(demoStoreService.changePassword(authorization, request));
    }
}
