package com.campus.demo.entity;

import com.campus.demo.enums.RoleCode;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
public class Role {

    private RoleCode code;
    private String name;
    private String description;
}
