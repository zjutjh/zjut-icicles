package com.campus.demo.controller;

import com.campus.demo.common.Result;
import com.campus.demo.entity.Role;
import com.campus.demo.service.DemoStoreService;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.util.List;

@RestController
@RequestMapping("/api/roles")
public class RoleController {

    private final DemoStoreService demoStoreService;

    public RoleController(DemoStoreService demoStoreService) {
        this.demoStoreService = demoStoreService;
    }

    @GetMapping
    public Result<List<Role>> listRoles() {
        return Result.ok(demoStoreService.listRoles());
    }
}
