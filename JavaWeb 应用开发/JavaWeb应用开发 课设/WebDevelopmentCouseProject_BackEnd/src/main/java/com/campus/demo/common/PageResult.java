package com.campus.demo.common;

import java.util.Collections;
import java.util.List;

public class PageResult<T> {

    private List<T> records;
    private Integer pageNo;
    private Integer pageSize;
    private Long total;
    private Long pages;

    public PageResult(List<T> records, Integer pageNo, Integer pageSize, Long total, Long pages) {
        this.records = records;
        this.pageNo = pageNo;
        this.pageSize = pageSize;
        this.total = total;
        this.pages = pages;
    }

    public static <T> PageResult<T> empty(Integer pageNo, Integer pageSize) {
        return new PageResult<>(Collections.emptyList(), pageNo, pageSize, 0L, 0L);
    }

    public List<T> getRecords() {
        return records;
    }

    public void setRecords(List<T> records) {
        this.records = records;
    }

    public Integer getPageNo() {
        return pageNo;
    }

    public void setPageNo(Integer pageNo) {
        this.pageNo = pageNo;
    }

    public Integer getPageSize() {
        return pageSize;
    }

    public void setPageSize(Integer pageSize) {
        this.pageSize = pageSize;
    }

    public Long getTotal() {
        return total;
    }

    public void setTotal(Long total) {
        this.total = total;
    }

    public Long getPages() {
        return pages;
    }

    public void setPages(Long pages) {
        this.pages = pages;
    }
}
