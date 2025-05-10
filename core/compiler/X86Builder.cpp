#include <cassert>
#include <cstring>

#include "X86Builder.h"

enum REX
{
    REX_B = 1 << 0,
    REX_X = 1 << 1,
    REX_R = 1 << 2,
    REX_W = 1 << 3,
};

// reg -> reg
void X86Builder::add(Reg32 dst, Reg32 src)
{
    add(RMOperand{dst}, src);
}

// reg -> reg, 16 bit
void X86Builder::add(Reg16 dst, Reg16 src)
{
    encode(0x00, RMOperand(dst), src);
}

// reg -> reg, 8bit
void X86Builder::add(Reg8 dst, Reg8 src)
{
    encode(0x00, RMOperand(dst), src);
}

void X86Builder::add(RMOperand dst, Reg32 src)
{
    encode(0x00, dst, src);
}

// imm -> reg, 32 bit
void X86Builder::add(Reg32 dst, uint32_t imm)
{
    add(RMOperand{dst}, imm);
}

// imm -> reg, 8 bit
void X86Builder::add(Reg8 dst, uint8_t imm)
{
    encode(0x80, 0, RMOperand(dst), imm);
}

void X86Builder::add(RMOperand dst, uint32_t imm)
{
    encode(0x80, 0, dst, imm);
}

// imm -> reg, 8 bit sign extended
void X86Builder::add(Reg64 dst, int8_t imm)
{
    encode(0x82, 0, RMOperand(dst), 64); // s = 1
    write(imm);
}

// imm -> reg, 8 bit sign extended
void X86Builder::add(Reg32 dst, int8_t imm)
{
    addD(RMOperand{dst}, imm);
}

// imm -> reg, 8 bit sign extended
void X86Builder::add(Reg16 dst, int8_t imm)
{
    addW(RMOperand{dst}, imm);
}

// imm -> mem, 32 bit mem, 8 bit sign extended
void X86Builder::addD(RMOperand dst, int8_t imm)
{
    encode(0x82, 0, RMOperand(dst), 32); // s = 1
    write(imm);
}

// imm -> mem, 16 bit mem, 8 bit sign extended
void X86Builder::addW(RMOperand dst, int8_t imm)
{
    encode(0x82, 0, RMOperand(dst), 16); // s = 1
    write(imm);
}

// reg -> reg
void X86Builder::adc(Reg32 dst, Reg32 src)
{
    adc(RMOperand(dst), src);
}

// reg -> reg, 8bit
void X86Builder::adc(Reg8 dst, Reg8 src)
{
    encode(0x10, RMOperand(dst), src);
}

void X86Builder::adc(RMOperand dst, Reg32 src)
{
    encode(0x10, dst, src);
}

// imm -> reg, 8 bit
void X86Builder::adc(Reg8 dst, uint8_t imm)
{
    encode(0x10, 2, RMOperand(dst), imm);
}

// reg -> reg
void X86Builder::and_(Reg32 dst, Reg32 src)
{
    and_(RMOperand(dst), src);
}

// reg -> reg, 16 bit
void X86Builder::and_(Reg16 dst, Reg16 src)
{
    encode(0x20, RMOperand(dst), src);
}

// reg -> reg, 8 bit
void X86Builder::and_(Reg8 dst, Reg8 src)
{
    encode(0x20, RMOperand(dst), src);
}

void X86Builder::and_(RMOperand dst, Reg32 src)
{
    encode(0x20, dst, src);
}

// imm -> reg
void X86Builder::and_(Reg32 dst, uint32_t imm)
{
    and_(RMOperand(dst), imm);
}

// imm -> reg, 8 bit
void X86Builder::and_(Reg8 dst, uint8_t imm)
{
    encode(0x80, 4, RMOperand(dst), imm);
}

void X86Builder::and_(RMOperand dst, uint32_t imm)
{
    encode(0x80, 4, RMOperand(dst), imm);
}

// imm -> reg, 8 bit sign extended
void X86Builder::and_(Reg32 dst, int8_t imm)
{
    andD(RMOperand(dst), imm);
}

void X86Builder::andD(RMOperand dst, int8_t imm)
{
    encode(0x82, 4, RMOperand(dst), 32); // s = 1
    write(imm);
}

void X86Builder::bt(Reg32 base, uint8_t off)
{
    auto baseReg = static_cast<int>(base);

    encodeREX(false, 0, 0, baseReg);
    write(0x0F); // two byte op
    write(0xBA); // opcode
    encodeModRM(baseReg, 4);
    write(off); // offset
}

void X86Builder::btr(Reg32 base, uint8_t off)
{
    auto baseReg = static_cast<int>(base);

    encodeREX(false, 0, 0, baseReg);
    write(0x0F); // two byte op
    write(0xBA); // opcode
    encodeModRM(baseReg, 6);
    write(off); // offset
}

// direct
void X86Builder::call(int disp)
{
    // adjust for opcode len
    if(disp < 0)
        disp -= 5;

    write(0xE8); // opcode

    write(disp);
    write(disp >> 8);
    write(disp >> 16);
    write(disp >> 24);
}

// indirect
void X86Builder::call(Reg64 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xFF); // opcode
    encodeModRM(reg, 2);
}

void X86Builder::cmc()
{
    write(0xF5); // opcode
}

// reg -> reg
void X86Builder::cmp(Reg32 dst, Reg32 src)
{
    encode(0x38, RMOperand(dst), src);
}

// reg -> reg, 8 bit
void X86Builder::cmp(Reg8 dst, Reg8 src)
{
    encode(0x38, RMOperand(dst), src);
}

// imm -> reg
void X86Builder::cmp(Reg32 dst, uint32_t imm)
{
    encode(0x80, 7, RMOperand(dst), imm);
}

// imm -> reg, 8 bit
void X86Builder::cmp(Reg8 dst, uint8_t imm)
{
    encode(0x80, 7, RMOperand(dst), imm);
}

// imm -> mem, 8 bit
void X86Builder::cmp(RMOperand dst, uint8_t imm)
{
    encode(0x80, 7, dst, imm);
}

// imm -> reg, 8 bit sign extended
void X86Builder::cmp(Reg32 dst, int8_t imm)
{
    cmp(RMOperand(dst), imm);
}

void X86Builder::cmp(RMOperand dst, int8_t imm)
{
    encode(0x82, 7, dst, 32); // s = 1
    write(imm);
}

// reg, 16 bit
void X86Builder::dec(Reg16 r)
{
    encode(0xFE, 1, RMOperand(r), 16);
}

// reg, 8 bit
void X86Builder::dec(Reg8 r)
{
    encode(0xFE, 1, RMOperand(r), 8);
}

// reg -> reg
void X86Builder::imul(Reg32 dst, Reg32 src)
{
    imul(dst, RMOperand(src));
}

void X86Builder::imul(Reg32 dst, RMOperand src)
{
    assert(src.w == 0 || src.w == 3);

    auto dstReg = static_cast<int>(dst);

    encodeREX(false, dstReg, src);
    write(0x0F); // two byte opcode
    write(0xAF); // opcode
    encodeModRM(src, dstReg);
}

// reg, 16 bit
void X86Builder::inc(Reg16 r)
{
    encode(0xFE, 0, RMOperand(r), 16);
}

// reg, 8 bit
void X86Builder::inc(Reg8 r)
{
    encode(0xFE, 1, RMOperand(r), 8);
}

void X86Builder::jcc(Condition cc, int disp)
{
    if(disp < 128 && disp >= -126) // 8 bit disp
    {
        // adjust for opcode len
        if(disp < 0)
            disp -= 2;

        write(0x70 | static_cast<int>(cc)); // opcode
        write(disp);
    }
    else
    {
        // adjust for opcode len
        if(disp < 0)
            disp -= 6;

        write(0x0F); // two byte opcode
        write(0x80 | static_cast<int>(cc)); // opcode

        write(disp);
        write(disp >> 8);
        write(disp >> 16);
        write(disp >> 24);
    }
}

void X86Builder::jmp(int disp, bool forceLong)
{
    if(disp < 128 && disp >= -126 && !forceLong) // 8 bit disp
    {
        // adjust for opcode len
        if(disp < 0)
            disp -= 2;

        write(0xEB); // opcode
        write(disp);
    }
    else
    {
        // adjust for opcode len
        if(disp < 0)
            disp -= 5;

        write(0xE9); // opcode

        write(disp);
        write(disp >> 8);
        write(disp >> 16);
        write(disp >> 24);
    }
}

// indirect
void X86Builder::jmp(Reg64 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xFF); // opcode
    encodeModRM(reg, 4);
}

void X86Builder::lea(Reg64 dst, RMOperand m)
{
    assert(m.w == 0); // not a register

    encode(0x8D, RMOperand(m), dst);
}

void X86Builder::lea(Reg32 dst, RMOperand m)
{
    assert(m.w == 0); // not a register

    encode(0x8D, RMOperand(m), dst);
}

// wrappers to make sure arg types are sensible
void X86Builder::mov(Reg64 dst, Reg64 src)
{
    mov(RMOperand{dst}, src);
}

void X86Builder::mov(Reg32 dst, Reg32 src)
{
    mov(RMOperand{dst}, src);
}

void X86Builder::mov(Reg16 dst, Reg16 src)
{
    mov(RMOperand{dst}, src);
}

void X86Builder::mov(Reg8 dst, Reg8 src)
{
    mov(RMOperand{dst}, src);
}

void X86Builder::mov(RMOperand dst, Reg64 src)
{
    encode(0x88, dst, src);
}

void X86Builder::mov(RMOperand dst, Reg32 src)
{
    encode(0x88, dst, src);
}

void X86Builder::mov(RMOperand dst, Reg16 src)
{
    encode(0x88, dst, src);
}

void X86Builder::mov(RMOperand dst, Reg8 src)
{
    encode(0x88, dst, src);
}

void X86Builder::mov(Reg64 dst, RMOperand src)
{
    encode(0x8A, src, dst);
}

void X86Builder::mov(Reg32 dst, RMOperand src)
{
    encode(0x8A, src, dst);
}

void X86Builder::mov(Reg16 dst, RMOperand src)
{
    encode(0x8A, src, dst);
}

void X86Builder::mov(Reg8 dst, RMOperand src)
{
    encode(0x8A, src, dst);
}

// imm -> reg, 64 bit
void X86Builder::mov(Reg64 r, uint64_t imm)
{
    auto reg = static_cast<int>(r);

    encodeREX(true, 0, 0, reg);
    write(0xB8 | (reg & 7)); // opcode

    // immediate
    write(imm);
    write(imm >> 8);
    write(imm >> 16);
    write(imm >> 24);
    write(imm >> 32);
    write(imm >> 40);
    write(imm >> 48);
    write(imm >> 56);
}

// imm -> reg
void X86Builder::mov(Reg32 r, uint32_t imm)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xB8 | (reg & 7)); // opcode

    // immediate
    write(imm);
    write(imm >> 8);
    write(imm >> 16);
    write(imm >> 24);
}

// imm -> reg, 8 bit
void X86Builder::mov(Reg8 r, uint8_t imm)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xB0 | (reg & 7)); // opcode

    // immediate
    write(imm);
}

// imm -> mem
void X86Builder::mov(RMOperand dst, uint32_t imm)
{
    encode(0xC6, 0, dst, imm);
}

// imm -> mem, 8 bit
void X86Builder::mov(RMOperand dst, uint8_t imm)
{
    encode(0xC6, 0, dst, imm);
}

// sign extend, reg -> reg, 16 bit
void X86Builder::movsx(Reg32 dst, Reg16 src)
{
    movsxW(dst, RMOperand(src));
}

// sign extend, reg -> reg, 8 bit
void X86Builder::movsx(Reg32 dst, Reg8 src)
{
    movsxB(dst, RMOperand(src));
}

void X86Builder::movsxW(Reg32 dst, RMOperand src)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, dstReg, src);
    write(0x0F); // two byte opcode
    write(0xBF); // opcode, w = 1
    encodeModRM(src, dstReg);
}

void X86Builder::movsxB(Reg32 dst, RMOperand src)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, dstReg, src);
    write(0x0F); // two byte opcode
    write(0xBE); // opcode, w = 0
    encodeModRM(src, dstReg);
}

// zero extend, reg -> reg, 16 bit
void X86Builder::movzx(Reg32 dst, Reg16 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, dstReg, 0, srcReg);
    write(0x0F); // two byte opcode
    write(0xB7); // opcode, w = 1
    encodeModRM(srcReg, dstReg);
}

// zero extend, reg -> reg, 8 bit
void X86Builder::movzx(Reg32 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, dstReg, 0, srcReg);
    write(0x0F); // two byte opcode
    write(0xB6); // opcode, w = 0
    encodeModRMReg8(srcReg, dstReg);
}

// zero extend, mem -> reg, 16 bit
void X86Builder::movzxW(Reg32 dst, RMOperand src)
{
    auto reg = static_cast<int>(dst);

    encodeREX(false, reg, src);
    write(0x0F); // two byte opcode
    write(0xB7); // opcode, w = 1
    encodeModRM(src, reg);
}

// reg
void X86Builder::not_(Reg32 r)
{
    encode(0xF6, 2, RMOperand(r), 32);
}

// reg, 8 bit
void X86Builder::not_(Reg8 r)
{
    encode(0xF6, 2, RMOperand(r), 32);
}

// reg -> reg
void X86Builder::or_(Reg32 dst, Reg32 src)
{
    or_(RMOperand(dst), src);
}

// reg -> reg, 16 bit
void X86Builder::or_(Reg16 dst, Reg16 src)
{
    encode(0x08, RMOperand(dst), src);
}

// reg -> reg, 8 bit
void X86Builder::or_(Reg8 dst, Reg8 src)
{
    encode(0x08, RMOperand(dst), src);
}

void X86Builder::or_(RMOperand dst, Reg32 src)
{
    encode(0x08, dst, src);
}

// imm -> reg
void X86Builder::or_(Reg32 dst, uint32_t imm)
{
    or_(RMOperand(dst), imm);
}

// imm -> reg, 8 bit
void X86Builder::or_(Reg8 dst, uint8_t imm)
{
    encode(0x80, 1, RMOperand(dst), imm);
}

void X86Builder::or_(RMOperand dst, uint32_t imm)
{
    encode(0x80, 1, RMOperand(dst), imm);
}

void X86Builder::pop(Reg64 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0x58 | (reg & 0x7)); // opcode
}

void X86Builder::push(Reg64 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0x50 | (reg & 0x7)); // opcode
}

// reg by CL, 8bit
void X86Builder::rcl(Reg8 r)
{
    encode(0xD2, 2, RMOperand(r), 8);
}

// reg, 8 bit
void X86Builder::rcl(Reg8 r, uint8_t count)
{
    encode(0xC0, 2, RMOperand(r), count);
}

// reg by CL, 8bit
void X86Builder::rcr(Reg8 r)
{
    encode(0xD2, 3, RMOperand(r), 8);
}

// reg, 8 bit
void X86Builder::rcr(Reg8 r, uint8_t count)
{
    encode(0xC0, 3, RMOperand(r), count);
}

void X86Builder::ret()
{
    write(0xC3); // opcode
}

// reg by CL, 8bit
void X86Builder::rol(Reg8 r)
{
    encode(0xD2, 0, RMOperand(r), 8);
}

// reg, 8 bit
void X86Builder::rol(Reg8 r, uint8_t count)
{
    encode(0xC0, 0, RMOperand(r), count);
}

// reg by CL
void X86Builder::ror(Reg32 r)
{
    rorD(RMOperand(r));
}

// reg by CL, 8bit
void X86Builder::ror(Reg8 r)
{
    encode(0xD2, 1, RMOperand(r), 8);
}

void X86Builder::rorD(RMOperand r)
{
    encode(0xD2, 1, r, 32);
}

// reg
void X86Builder::ror(Reg32 r, uint8_t count)
{
    rorD(RMOperand(r), count);
}

// reg, 8 bit
void X86Builder::ror(Reg8 r, uint8_t count)
{
    encode(0xC0, 1, RMOperand(r), count);
}

void X86Builder::rorD(RMOperand r, uint8_t count)
{
    encode(0xC0, 1, r, 32);
    write(count);
}

// reg by CL
void X86Builder::sar(Reg32 r)
{
    sarD(RMOperand(r));
}

// reg by CL, 8bit
void X86Builder::sar(Reg8 r)
{
    encode(0xD2, 7, RMOperand(r), 8);
}

void X86Builder::sarD(RMOperand r)
{
    encode(0xD2, 7, r, 32);
}

// reg
void X86Builder::sar(Reg32 r, uint8_t count)
{
    sarD(RMOperand(r), count);
}

// reg, 8 bit
void X86Builder::sar(Reg8 r, uint8_t count)
{
    // TODO: 0xD0 for 1
    encode(0xC0, 7, RMOperand(r), count);
}

void X86Builder::sarD(RMOperand r, uint8_t count)
{
    // TODO: 0xD0 for 1
    encode(0xC0, 7, r, 32);
    write(count);
}

// reg -> reg
void X86Builder::sbb(Reg32 dst, Reg32 src)
{
    sbb(RMOperand(dst), src);
}

// reg -> reg, 8bit
void X86Builder::sbb(Reg8 dst, Reg8 src)
{
    encode(0x18, RMOperand(dst), src);
}

void X86Builder::sbb(RMOperand dst, Reg32 src)
{
    encode(0x18, dst, src);
}

// imm -> reg, 8 bit
void X86Builder::sbb(Reg8 dst, uint8_t imm)
{
    encode(0x80, 3, RMOperand(dst), imm);
}

// -> reg
void X86Builder::setcc(Condition cc, Reg8 dst)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x0F); // two byte opcode
    write(0x90 | static_cast<int>(cc)); // opcode
    encodeModRM(dstReg);
}

// reg by CL
void X86Builder::shr(Reg32 r)
{
    shrD(RMOperand(r));
}

// reg by CL, 8bit
void X86Builder::shr(Reg8 r)
{
    encode(0xD2, 5, RMOperand(r), 8);
}

void X86Builder::shrD(RMOperand r)
{
    encode(0xD2, 5, r, 32);
}

// reg
void X86Builder::shr(Reg32 r, uint8_t count)
{
    shrD(RMOperand(r), count);
}

// reg, 8 bit
void X86Builder::shr(Reg8 r, uint8_t count)
{
    encode(0xC0, 5, RMOperand(r), count);
}

void X86Builder::shrD(RMOperand r, uint8_t count)
{
    encode(0xC0, 5, r, 32);
    write(count);
}

// reg by CL
void X86Builder::shl(Reg32 r)
{
    shlD(RMOperand(r));
}

// reg by CL, 8bit
void X86Builder::shl(Reg8 r)
{
    encode(0xD2, 4, RMOperand(r), 8);
}

void X86Builder::shlD(RMOperand r)
{
    encode(0xD2, 4, r, 32);
}

// reg
void X86Builder::shl(Reg32 r, uint8_t count)
{
    shlD(RMOperand(r), count);
}

// reg, 8 bit
void X86Builder::shl(Reg8 r, uint8_t count)
{
    // TODO: 0xD0 for 1
    encode(0xC0, 4, RMOperand(r), count);
}

void X86Builder::shlD(RMOperand r, uint8_t count)
{
    // TODO: 0xD0 for 1
    encode(0xC0, 4, r, 32);
    write(count);
}

void X86Builder::stc()
{
    write(0xF9); // opcode
}

// reg -> reg
void X86Builder::sub(Reg32 dst, Reg32 src)
{
    sub(RMOperand{dst}, src);
}

// reg -> reg, 16 bit
void X86Builder::sub(Reg16 dst, Reg16 src)
{
    encode(0x28, RMOperand(dst), src);
}

// reg -> reg, 8bit
void X86Builder::sub(Reg8 dst, Reg8 src)
{
    encode(0x28, RMOperand(dst), src);
}

void X86Builder::sub(RMOperand dst, Reg32 src)
{
    encode(0x28, dst, src);
}

// imm -> reg, 32 bit
void X86Builder::sub(Reg32 dst, uint32_t imm)
{
    sub(RMOperand{dst}, imm);
}

// imm -> reg, 8 bit
void X86Builder::sub(Reg8 dst, uint8_t imm)
{
    encode(0x80, 5, RMOperand(dst), imm);
}

void X86Builder::sub(RMOperand dst, uint32_t imm)
{
    encode(0x80, 5, dst, imm);
}

// imm -> reg, 64 bit reg, 8 bit sign extended
void X86Builder::sub(Reg64 dst, int8_t imm)
{
    encode(0x82, 5, RMOperand(dst), 64); // s = 1
    write(imm);
}

// imm -> reg, 8 bit sign extended
void X86Builder::sub(Reg32 dst, int8_t imm)
{
    subD(RMOperand{dst}, imm);
}

// imm -> mem, 32 bit mem, 8 bit sign extended
void X86Builder::subD(RMOperand dst, int8_t imm)
{
    encode(0x82, 5, dst, 32); // s = 1
    write(imm);
}

// imm -> reg
void X86Builder::test(Reg32 dst, uint32_t imm)
{
    encode(0xF6, 0, RMOperand(dst), imm);
}

// imm -> reg, 8 bit
void X86Builder::test(Reg8 r, uint8_t imm)
{
    encode(0xF6, 0, RMOperand(r), imm);
}

// reg -> reg
void X86Builder::xchg(Reg32 dst, Reg32 src)
{
    xchg(RMOperand(dst), src);
}

// reg -> reg, 8bit
void X86Builder::xchg(Reg8 dst, Reg8 src)
{
    encode(0x86, RMOperand(dst), src);
}

void X86Builder::xchg(RMOperand dst, Reg32 src)
{
    encode(0x86, dst, src);
}

// reg -> reg
void X86Builder::xor_(Reg32 dst, Reg32 src)
{
    xor_(RMOperand(dst), src);
}

// reg -> reg, 8 bit
void X86Builder::xor_(Reg8 dst, Reg8 src)
{
    encode(0x30, RMOperand(dst), src);
}

void X86Builder::xor_(RMOperand dst, Reg32 src)
{
    encode(0x30, dst, src);
}

// imm -> reg
void X86Builder::xor_(Reg32 dst, uint32_t imm)
{
    xor_(RMOperand(dst), imm);
}

// imm -> reg, 8 bit
void X86Builder::xor_(Reg8 dst, uint8_t imm)
{
    encode(0x80, 6, RMOperand(dst), imm);
}

void X86Builder::xor_(RMOperand dst, uint32_t imm)
{
    encode(0x80, 6, dst, imm);
}

void X86Builder::resetPtr(uint8_t *oldPtr)
{
    assert(oldPtr <= ptr);
    ptr = oldPtr;
    error = false;
}

void X86Builder::patch(uint8_t *patchPtr, uint8_t *patchEndPtr)
{
    assert(patchPtr < ptr);
    assert(patchEndPtr < ptr);

    savedPtr = ptr;
    savedEndPtr = endPtr;

    ptr = patchPtr;
    endPtr = patchEndPtr;
}

void X86Builder::endPatch()
{
    ptr = savedPtr;
    endPtr = savedEndPtr;
}

void X86Builder::removeRange(uint8_t *startPtr, uint8_t *endPtr)
{
    if(error)
        return;

    // making sure this doesn't break any jumps is the user's problem

    assert(startPtr < ptr && endPtr <= ptr);
    assert(endPtr > startPtr);

    // move any code after the end of the removed range
    if(endPtr != ptr)
        memmove(startPtr, endPtr, ptr - endPtr);

    this->ptr -= (endPtr - startPtr);
}

void X86Builder::write(uint8_t b)
{
    if(ptr != endPtr)
        *ptr++ = b;
    else
        error = true;
}

void X86Builder::encode(uint8_t opcode, int regOp, RMOperand rm, int width, bool isReg)
{
    // make sure register sizes are consistent
    // (unless one is memory)
    assert(rm.w == 0 || (4 << rm.w) == width);

    // make sure width is sensible
    assert(width == 8 || width == 16 || width == 32 || width == 64);

    if(width == 16)
        write(0x66); // 16 bit override

    encodeREX(width == 64, regOp, rm);
    write(opcode | (width > 8 ? 1 : 0)); // set w bit if not 8bit
    encodeModRM(rm, regOp, isReg);
}

void X86Builder::encode(uint8_t opcode, int subOp, RMOperand rm, uint32_t imm)
{
    encode(opcode, subOp, rm, 32);

    // immediate
    write(imm);
    write(imm >> 8);
    write(imm >> 16);
    write(imm >> 24);
}

void X86Builder::encode(uint8_t opcode, int subOp, RMOperand rm, uint8_t imm)
{
    encode(opcode, subOp, rm, 8);
    write(imm);
}

void X86Builder::encodeModRM(int reg1, int reg2Op)
{
    write(0xC0 | (reg2Op & 7) << 3 | (reg1 & 7)); // mod = 3, reg 2 or sub-opcode, reg 1
}

void X86Builder::encodeModRM(RMOperand rm, int reg2Op, bool isReg)
{
    auto baseReg = static_cast<int>(rm.base);
    if(rm.w) // direct reg
    {
        if(rm.w == 1 && isReg)
        {
            // if one reg is >= 8, the other can't be xH (4-7)
            assert((reg2Op < 8 || baseReg >= 8 || baseReg < 4) && (baseReg < 8 || reg2Op >= 8 || reg2Op < 4));
        }

        write(0xC0 | (reg2Op & 7) << 3 | (baseReg & 7)); // mod = 3, reg 2 or sub-opcode, reg 1
        return;
    }

    int mod = 0; // no disp
    int rmVal = baseReg & 7;
    
    if(rm.disp || rmVal == 5 /*BP*/)
        mod = (rm.disp >= 128 || rm.disp < -128) ? 2 : 1; // 32-bit or 8-bit disp

    if(rm.scale)
        rmVal = 4; // use SIB

    write((mod << 6) | ((reg2Op & 7) << 3) | rmVal); // write ModRM byte

    if(rmVal == 4) // need SIB (base is SP)
    {
        // scale
        int ss = 0;
        if(rm.scale == 2)
            ss = 1;
        else if(rm.scale == 4)
            ss = 2;
        else if(rm.scale == 8)
            ss = 3;

        write(ss << 6 | (static_cast<int>(rm.index) & 7) << 3 | (baseReg & 7)); // write SIB
    }

    // write disp
    if(mod == 1)
        write(rm.disp);
    else if(mod == 2)
    {
        write(rm.disp);
        write(rm.disp >> 8);
        write(rm.disp >> 16);
        write(rm.disp >> 24);
    }
}

void X86Builder::encodeModRMReg8(int reg1, int reg2)
{
    // if one reg is >= 8, the other can't be xH (4-7)
    assert((reg2 < 8 || reg1 >= 8 || reg1 < 4) && (reg1 < 8 || reg2 >= 8 || reg2 < 4));

    encodeModRM(reg1, reg2);
}

void X86Builder::encodeREX(bool w, int reg, int index, int base)
{
    if(!w && reg <= 7 && index <= 7 && base <= 7)
        return;

    write(0x40 | (w ? REX_W : 0)
               | (reg & 8 ? REX_R : 0)
               | (index & 8 ? REX_X : 0)
               | (base & 8 ? REX_B : 0));
}

void X86Builder::encodeREX(bool w, int reg, RMOperand rm)
{
    encodeREX(w, reg, static_cast<int>(rm.index), static_cast<int>(rm.base));
}