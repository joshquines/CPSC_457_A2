// AVL-tree implementation
// source: https://github.com/SuprDewd/CompetitiveProgramming/blob/master/code/data-structures/avl_tree.cpp
// note that the code on the github is very unreadable
// so most of the changes are removals of unused things,
// additions of the min node functions
// and making the code more readable.


// This code is by a student who was enrolled in CPSC 457 in Fall 2015
//#include <assert.h>
template <class T>
class Tree {
public:
    struct node {
        T item; node *p, *l, *r;
        int size, height;
        node(const T &_item, node *_p = nullptr) : item(_item), p(_p),
        l(nullptr), r(nullptr), size(1), height(0) { } };
    Tree() : root(nullptr) { }
    node *root;
    
    bool empty() const {
        return root==nullptr;
    }
    
    T* readMinNode() const {
        node *cur = root;
        while (cur->l){ cur=cur->l; }
        return &(cur->item);
    }
    
    T* popMinNode() {
        node *cur = root;
        while (cur->l){ cur=cur->l; }
        T* item = &(cur->item);
        erase(cur);
        return item;
    }
    
    node* find(const T &item) const {
        node *cur = root;
        while (cur) {
            if (cur->item < item) cur = cur->r;
            else if (item < cur->item) cur = cur->l;
            else break; }
        return cur; }
        
    void insert(const T &item) {
        node *prev = nullptr, **cur = &root;
        while (*cur) {
            prev = *cur;
            if ((*cur)->item < item) cur = &((*cur)->r);
            else cur = &((*cur)->l);
        }
        node *n = new node(item, prev);
        *cur = n, fix(n);
        return;
    }
    
    void deleteNode(const T &item) {erase(find(item));}
    void erase(node *n) {
        if (!n) return;
        if (!n->l && n->r) parent_leg(n) = n->r, n->r->p = n->p;
        else if (n->l && !n->r) parent_leg(n) = n->l, n->l->p = n->p;
        else if (n->l && n->r) {
            node *s = successor(n);
            erase(s);
            s->p = n->p, s->l = n->l, s->r = n->r;
            if (n->l) n->l->p = s;
            if (n->r) n->r->p = s;
            parent_leg(n) = s, fix(s);
            return;
        } else parent_leg(n) = nullptr;
        fix(n->p), n->p = n->l = n->r = nullptr;
    }
    
    node* successor(node *n) const {
        if (!n) return nullptr;
        if (n->r) return nth(0, n->r);
        node *p = n->p;
        while (p && p->r == n) n = p, p = p->p;
        return p;
    }
    
    node* nth(int n, node *cur = nullptr) const {
        if (!cur) cur = root;
        while (cur) {
            if (n < sz(cur->l)) cur = cur->l;
            else if (n > sz(cur->l)){
             n -= sz(cur->l) + 1; cur = cur->r;
            }  
            else break;
        } return cur; }

private:
    inline int sz(node *n) const { return n ? n->size : 0; }
    inline int height(node *n) const { return n ? n->height : -1; }
    inline bool left_heavy(node *n) const {
        return n && height(n->l) > height(n->r); }
    inline bool right_heavy(node *n) const {
        return n && height(n->r) > height(n->l); }
    inline bool too_heavy(node *n) const {
        int hl = height(n->l);
        int hr = height(n->r);
        int m = hl>hr ? hl-hr : hr-hl;
        return n && m > 1;
    }

    node*& parent_leg(node *n) {
        if (!n->p) return root;
        if (n->p->l == n) return n->p->l;
        if (n->p->r == n) return n->p->r;
        //assert(false);
        }
    
    void augment(node *n) {
        if (!n) return;
        n->size = 1 +  sz(n->l) + sz(n->r);
        int hl = height(n->l);
        int hr = height(n->r);
        int m = hl>hr ? hl : hr;
        n->height = 1 + m;
    }
    #define rotate(l, r) \
        node *l = n->l; \
        l->p = n->p; \
        parent_leg(n) = l; \
        n->l = l->r; \
        if (l->r) l->r->p = n; \
        l->r = n, n->p = l; \
        augment(n), augment(l)
    void left_rotate(node *n) { rotate(r, l); }
    void right_rotate(node *n) { rotate(l, r); }
    void fix(node *n) {
        while (n) { augment(n);
            if (too_heavy(n)) {
                if (left_heavy(n) && right_heavy(n->l)) left_rotate(n->l);
                else if (right_heavy(n) && left_heavy(n->r))
                    right_rotate(n->r);
                if (left_heavy(n)) right_rotate(n);
                else left_rotate(n);
                n = n->p; }
            n = n->p; } }
};


#include "runtime/Thread.h"
// Class for tree node
class ThreadNode{
    friend class Scheduler;
    Thread *th;
    
    public:
        bool operator < (ThreadNode other) const {
            return th->priority < other.th->priority;
        }
        bool operator == (ThreadNode other) const {
            return th->priority == other.th->priority;
        }
        bool operator > (ThreadNode other) const {
            return th->priority > other.th->priority;
        }
    
    //this is how we want to do it
    ThreadNode(Thread *t){
        th = t;
    }
};