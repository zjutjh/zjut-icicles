package com.campus.demo.service;

import com.campus.demo.dto.CreateUserRequest;
import com.campus.demo.entity.SysUser;

import java.util.List;

public interface UserService {

    SysUser getById(Long id);

    List<SysUser> listUsers();

    List<SysUser> searchByRole(String roleCode);

    SysUser createUser(CreateUserRequest request);
}
