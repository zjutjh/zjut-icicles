package com.campus.demo.entity;

import com.campus.demo.enums.RoleCode;
import lombok.Data;

import java.time.LocalDateTime;

@Data
public class AssessmentScoreDetail {

    private Long id;
    private Long assessmentId;
    private Long scorerUserId;
    private String scorerName;
    private RoleCode scorerRole;
    private Double rawScore;
    private String comment;
    private LocalDateTime scoreTime;
}
