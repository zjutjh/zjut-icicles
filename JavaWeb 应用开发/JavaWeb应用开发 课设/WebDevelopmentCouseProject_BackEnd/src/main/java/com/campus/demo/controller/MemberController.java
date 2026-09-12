package com.campus.demo.controller;

import com.campus.demo.common.PageResult;
import com.campus.demo.common.Result;
import com.campus.demo.dto.CreateMemberRequest;
import com.campus.demo.dto.MemberStatusUpdateRequest;
import com.campus.demo.dto.UpdateMemberRequest;
import com.campus.demo.entity.Member;
import com.campus.demo.enums.MemberStatus;
import com.campus.demo.service.DemoStoreService;
import jakarta.validation.Valid;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/api/members")
public class MemberController {

    private final DemoStoreService demoStoreService;

    public MemberController(DemoStoreService demoStoreService) {
        this.demoStoreService = demoStoreService;
    }

    @GetMapping
    public Result<PageResult<Member> > listMembers(
            @RequestParam(required = false) String keyword,
            @RequestParam(required = false) Long teamId,
            @RequestParam(required = false) MemberStatus status,
            @RequestParam(required = false) Integer pageNo,
            @RequestParam(required = false) Integer pageSize) {
        return Result.ok(demoStoreService.listMembers(keyword, teamId, status, pageNo, pageSize));
    }

    @GetMapping("/{memberId}")
    public Result<Member> getMember(@PathVariable Long memberId) {
        return Result.ok(demoStoreService.getMember(memberId));
    }

    @PostMapping
    public Result<Member> createMember(@Valid @RequestBody CreateMemberRequest request) {
        return Result.ok(demoStoreService.createMember(request));
    }

    @PutMapping("/{memberId}")
    public Result<Member> updateMember(@PathVariable Long memberId, @Valid @RequestBody UpdateMemberRequest request) {
        return Result.ok(demoStoreService.updateMember(memberId, request));
    }

    @PatchMapping("/{memberId}/status")
    public Result<Member> updateMemberStatus(@PathVariable Long memberId, @Valid @RequestBody MemberStatusUpdateRequest request) {
        return Result.ok(demoStoreService.updateMemberStatus(memberId, request));
    }
}
