package com.campus.demo.entity;

import com.campus.demo.enums.AssessmentStatus;
import lombok.Data;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
public class MonthlyAssessment {

    private Long id;
    private Long memberId;
    private String memberName;
    private Long teamId;
    private String teamName;
    private String assessMonth;
    private Double deptManagerScore;
    private Double techDirectorScore;
    private Double generalManagerScore;
    private Double finalScore;
    private String ratingLevel;
    private AssessmentStatus status;
    private String comment;
    private List<AssessmentScoreDetail> scoreDetails = new ArrayList<>();
    private LocalDateTime createdAt;
    private LocalDateTime updatedAt;
}
