package com.campus.demo.entity;

import com.campus.demo.enums.MemberStatus;
import lombok.Data;

import java.time.LocalDate;

@Data
public class Member {

    private Long id;
    private String name;
    private Long teamId;
    private String teamName;
    private String position;
    private String jobLevel;
    private String phone;
    private String phoneMasked;
    private String email;
    private MemberStatus status;
    private LocalDate entryDate;
}
