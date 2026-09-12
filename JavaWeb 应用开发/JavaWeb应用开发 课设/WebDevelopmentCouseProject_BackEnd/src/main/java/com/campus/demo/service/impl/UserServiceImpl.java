package com.campus.demo.service.impl;

import com.campus.demo.dto.CreateUserRequest;
import com.campus.demo.entity.SysUser;
import com.campus.demo.enums.RoleCode;
import com.campus.demo.service.DemoStoreService;
import com.campus.demo.service.UserService;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
public class UserServiceImpl implements UserService {

    private final DemoStoreService demoStoreService;

    public UserServiceImpl(DemoStoreService demoStoreService) {
        this.demoStoreService = demoStoreService;
    }

    @Override
    public SysUser getById(Long id) {
        return demoStoreService.getUser(id);
    }

    @Override
    public List<SysUser> listUsers() {
        return demoStoreService.listUsers(null, null, null, 1, Integer.MAX_VALUE).getRecords();
    }

    @Override
    public List<SysUser> searchByRole(String roleCode) {
        return demoStoreService.listUsers(null, RoleCode.valueOf(roleCode), null, 1, Integer.MAX_VALUE).getRecords();
    }

    @Override
    public SysUser createUser(CreateUserRequest request) {
        return demoStoreService.createUser(request);
    }
}
