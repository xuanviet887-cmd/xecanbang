#ifndef FUZZY_CONTROL_H
#define FUZZY_CONTROL_H

#include <Fuzzy.h>

// Đối tượng Fuzzy
Fuzzy *fuzzy = new Fuzzy();

// Các biến ngôn ngữ
// Input 1: Error (Sai số góc)
FuzzyInput *error = new FuzzyInput(1);
// Input 2: dError (Đạo hàm sai số/Vận tốc góc)
FuzzyInput *dError = new FuzzyInput(2);
// Output: PWM (Tốc độ động cơ)
FuzzyOutput *pwm = new FuzzyOutput(1);

void setup_fuzzy() {
    // --- Cấu hình Input: Error (Phạm vi -20 đến 20 độ) ---
    FuzzySet *error_nb = new FuzzySet(-20, -20, -15, -10); // Negative Big
    FuzzySet *error_ns = new FuzzySet(-15, -10, -5, 0);    // Negative Small
    FuzzySet *error_ze = new FuzzySet(-5, 0, 0, 5);       // Zero (Mượt mà ở đây)
    FuzzySet *error_ps = new FuzzySet(0, 5, 10, 15);      // Positive Small
    FuzzySet *error_pb = new FuzzySet(10, 15, 20, 20);    // Positive Big
    
    error->addFuzzySet(error_nb);
    error->addFuzzySet(error_ns);
    error->addFuzzySet(error_ze);
    error->addFuzzySet(error_ps);
    error->addFuzzySet(error_pb);
    fuzzy->addFuzzyInput(error);

    // --- Cấu hình Input: dError (Vận tốc góc -100 đến 100 deg/s) ---
    FuzzySet *dError_neg = new FuzzySet(-100, -100, -50, 0);
    FuzzySet *dError_ze = new FuzzySet(-50, 0, 0, 50);
    FuzzySet *dError_pos = new FuzzySet(0, 50, 100, 100);
    
    dError->addFuzzySet(dError_neg);
    dError->addFuzzySet(dError_ze);
    dError->addFuzzySet(dError_pos);
    fuzzy->addFuzzyInput(dError);

    // --- Cấu hình Output: PWM (-255 đến 255) ---
    FuzzySet *pwm_nb = new FuzzySet(-255, -255, -200, -100);
    FuzzySet *pwm_ns = new FuzzySet(-150, -100, -50, 0);
    FuzzySet *pwm_ze = new FuzzySet(-20, 0, 0, 20); // Dải chết nhỏ để mượt
    FuzzySet *pwm_ps = new FuzzySet(0, 50, 100, 150);
    FuzzySet *pwm_pb = new FuzzySet(100, 200, 255, 255);
    
    pwm->addFuzzySet(pwm_nb);
    pwm->addFuzzySet(pwm_ns);
    pwm->addFuzzySet(pwm_ze);
    pwm->addFuzzySet(pwm_ps);
    pwm->addFuzzySet(pwm_pb);
    fuzzy->addFuzzyOutput(pwm);

    // --- BẢNG LUẬT MỜ (Rules) để di chuyển mượt ---
    // 1. Nếu Error Zero và dError Zero -> PWM Zero
    FuzzyRuleAntecedent *ifErrorZeAndDerrorZe = new FuzzyRuleAntecedent();
    ifErrorZeAndDerrorZe->joinWithAND(error_ze, dError_ze);
    FuzzyRuleConsequent *thenPwmZe = new FuzzyRuleConsequent();
    thenPwmZe->addOutput(pwm_ze);
    FuzzyRule *rule1 = new FuzzyRule(1, ifErrorZeAndDerrorZe, thenPwmZe);
    fuzzy->addFuzzyRule(rule1);

    // 2. Nếu Error Positive Small (Nghiêng tới nhẹ) -> PWM Positive Small
    FuzzyRuleAntecedent *ifErrorPs = new FuzzyRuleAntecedent();
    ifErrorPs->joinSingle(error_ps);
    FuzzyRuleConsequent *thenPwmPs = new FuzzyRuleConsequent();
    thenPwmPs->addOutput(pwm_ps);
    FuzzyRule *rule2 = new FuzzyRule(2, ifErrorPs, thenPwmPs);
    fuzzy->addFuzzyRule(rule2);

    // 3. Nếu Error Negative Small (Nghiêng lui nhẹ) -> PWM Negative Small
    FuzzyRuleAntecedent *ifErrorNs = new FuzzyRuleAntecedent();
    ifErrorNs->joinSingle(error_ns);
    FuzzyRuleConsequent *thenPwmNs = new FuzzyRuleConsequent();
    thenPwmNs->addOutput(pwm_ns);
    FuzzyRule *rule3 = new FuzzyRule(3, ifErrorNs, thenPwmNs);
    fuzzy->addFuzzyRule(rule3);

    // 4. Nếu Error Positive Big (Sắp đổ tới) -> PWM Positive Big (Phản ứng nhanh)
    FuzzyRuleAntecedent *ifErrorPb = new FuzzyRuleAntecedent();
    ifErrorPb->joinSingle(error_pb);
    FuzzyRuleConsequent *thenPwmPb = new FuzzyRuleConsequent();
    thenPwmPb->addOutput(pwm_pb);
    FuzzyRule *rule4 = new FuzzyRule(4, ifErrorPb, thenPwmPb);
    fuzzy->addFuzzyRule(rule4);

    // 5. Nếu Error Negative Big (Sắp đổ lui) -> PWM Negative Big
    FuzzyRuleAntecedent *ifErrorNb = new FuzzyRuleAntecedent();
    ifErrorNb->joinSingle(error_nb);
    FuzzyRuleConsequent *thenPwmNb = new FuzzyRuleConsequent();
    thenPwmNb->addOutput(pwm_nb);
    FuzzyRule *rule5 = new FuzzyRule(5, ifErrorNb, thenPwmNb);
    fuzzy->addFuzzyRule(rule5);
}

#endif
