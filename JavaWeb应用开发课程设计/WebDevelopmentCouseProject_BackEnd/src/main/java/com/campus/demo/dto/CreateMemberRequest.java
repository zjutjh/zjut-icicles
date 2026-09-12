package com.campus.demo.dto;

import jakarta.validation.constraints.NotBlank;
import lombok.Data;

import java.time.LocalDate;

@Data
public class CreateMemberRequest {

    @NotBlank(message = "name 不能为空")
    private String name;

    private Long teamId;
    private String position;
    private String jobLevel;
    private String phone;
    private String email;
    private LocalDate entryDate;
}
