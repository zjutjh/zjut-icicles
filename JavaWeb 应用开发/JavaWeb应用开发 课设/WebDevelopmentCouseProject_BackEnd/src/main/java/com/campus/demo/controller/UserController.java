package com.campus.demo.controller;

import com.campus.demo.common.PageResult;
import com.campus.demo.common.Result;
import com.campus.demo.dto.CreateUserRequest;
import com.campus.demo.dto.UpdateUserRequest;
import com.campus.demo.dto.UserStatusUpdateRequest;
import com.campus.demo.entity.SysUser;
import com.campus.demo.enums.RoleCode;
import com.campus.demo.enums.UserStatus;
import com.campus.demo.service.DemoStoreService;
import jakarta.validation.Valid;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/api/users")
public class UserController {

    private final DemoStoreService demoStoreService;

    public UserController(DemoStoreService demoStoreService) {
        this.demoStoreService = demoStoreService;
    }

    @GetMapping
    public Result<PageResult<SysUser> > listUsers(
            @RequestParam(required = false) String keyword,
            @RequestParam(required = false) RoleCode roleCode,
            @RequestParam(required = false) UserStatus status,
            @RequestParam(required = false) Integer pageNo,
            @RequestParam(required = false) Integer pageSize) {
        return Result.ok(demoStoreService.listUsers(keyword, roleCode, status, pageNo, pageSize));
    }

    @GetMapping("/{userId}")
    public Result<SysUser> getUser(@PathVariable Long userId) {
        return Result.ok(demoStoreService.getUser(userId));
    }

    @PostMapping
    public Result<SysUser> createUser(@Valid @RequestBody CreateUserRequest request) {
        return Result.ok(demoStoreService.createUser(request));
    }

    @PutMapping("/{userId}")
    public Result<SysUser> updateUser(@PathVariable Long userId, @Valid @RequestBody UpdateUserRequest request) {
        return Result.ok(demoStoreService.updateUser(userId, request));
    }

    @PatchMapping("/{userId}/status")
    public Result<SysUser> updateUserStatus(@PathVariable Long userId, @Valid @RequestBody UserStatusUpdateRequest request) {
        return Result.ok(demoStoreService.updateUserStatus(userId, request));
    }
}
