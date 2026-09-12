package com.campus.demo.entity;

import lombok.Data;

import java.time.LocalDateTime;

@Data
public class TaskProgress {

    private Long id;
    private Long taskId;
    private Integer progressRate;
    private String issueDesc;
    private String comment;
    private LocalDateTime updateTime;
}
