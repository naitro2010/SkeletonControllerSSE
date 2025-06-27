#include "GameEventHandler.h"
#include "Hooks.h"
#include <detours/detours.h>
#include "include/mutil/mutil.h"
namespace plugin {
    void UpdateClavicle(RE::Actor* obj, float delta); 
    void* OriginalActorUpdatePtr = nullptr;
    void* OriginalPlayerUpdatePtr = nullptr;
    void* OriginalPlayerUpdate3PPtr = nullptr;
    RE::NiAVObject* FindNiObjectName(RE::NiNode* obj, std::string FindName) {
        if (!obj) {
            return nullptr;
        }
        for (auto aviter: obj->GetChildren()) {
            if (!aviter) {
                continue;
            }
            logger::info("{}", aviter->name.c_str());
            if (aviter->name.contains(FindName)) {
                return aviter.get();
            }
            if (aviter->AsNode()) {
                if (RE::NiAVObject* found = FindNiObjectName(aviter->AsNode(), FindName)) {
                    return found;
                }
            }
        }
        return nullptr;
    }
    inline RE::NiMatrix3 add_matrix(RE::NiMatrix3 A, RE::NiMatrix3 B) {
        RE::NiMatrix3 sum;
        for (int x = 0; x < 3; x++) {
            for (int y = 0; y < 3; y++) {
                sum.entry[x][y] = A.entry[x][y] + B.entry[x][y];
            }
        }
    }
    void ActorUpdateHook(RE::Actor* obj, float delta) {
        auto OriginalUpdate = (void (*)(RE::Actor* obj, float delta)) OriginalActorUpdatePtr;
        OriginalUpdate(obj, delta);
        UpdateClavicle(obj, delta);
    }
    void PlayerUpdateHook(RE::Actor* obj, float delta) {
        auto OriginalUpdate = (void (*)(RE::Actor* obj, float delta)) OriginalPlayerUpdatePtr;
        OriginalUpdate(obj, delta);
        UpdateClavicle(obj, delta);
    }
    void PlayerUpdate3PHook(RE::Actor* obj) {
        auto OriginalUpdate = (void (*)(RE::Actor* obj)) OriginalPlayerUpdate3PPtr;
        UpdateClavicle(obj, 0.0f);
        OriginalUpdate(obj);
        
    }
    void UpdateClavicle(RE::Actor* obj, float delta) 
    {

        if (!obj->Is3DLoaded() || !obj->GetCurrent3D() || !obj->GetCurrent3D()->AsNode()) {
            return;
        }
        std::vector<std::string> nodes;
        
        nodes.push_back("NPC Clavicle2.L");
        nodes.push_back("NPC L Clavicle [LClv]");
        nodes.push_back("NPC L UpperArm [LUar]");
        nodes.push_back("NPC Clavicle2.R");
        nodes.push_back("NPC R Clavicle [RClv]");
        nodes.push_back("NPC R UpperArm [RUar]");
        RE::NiPoint3 UAL_pos(-0.000014f, -0.000005f, 14.043962f);
        RE::NiPoint3 C2L_pos(10.534419f, 1.130187f, 14.431618f);
        RE::NiPoint3 UAR_pos(-0.000003f, -0.000010f, 14.043943f);
        RE::NiPoint3 C2R_pos(-10.534427f, 1.130133f, 14.431620f);
        mutil::Quaternion original_C2L_quat;
        original_C2L_quat.w = 0.314607;
        original_C2L_quat.x = 0.667405;
        original_C2L_quat.y = -0.590508;
        original_C2L_quat.z = 0.326944;
        mutil::Quaternion original_C2R_quat;
        original_C2R_quat.w = 0.314607;
        original_C2R_quat.x = 0.667405;
        original_C2R_quat.y = 0.590508;
        original_C2R_quat.z = -0.326944;
        for (int r = 0; r < 2; r++) {
            
                auto clavicle2 = FindNiObjectName(obj->GetCurrent3D()->AsNode(), nodes[0 + r * 3].c_str());
                if (!clavicle2) {
                    continue;
                }
                auto clavicle2node = clavicle2->AsNode();
                auto clavicle = FindNiObjectName(clavicle2->parent->AsNode(), nodes[1 + r * 3].c_str());
                auto upperarm = FindNiObjectName(clavicle2->parent->AsNode(), nodes[2 + r * 3].c_str());
                auto shouldernode = clavicle2->parent->AsNode();
                // original positions from NIF
                auto UA_pos = UAL_pos;
                auto C2_pos = C2L_pos;
                auto original_C2_quat = original_C2L_quat;
                if (r == 1) {
                    UA_pos = UAR_pos;
                    C2_pos = C2R_pos;
                    original_C2_quat = original_C2R_quat;
                }
                RE::NiPoint3 original_C2_vector = upperarm->parent->world * UA_pos - clavicle2->parent->world * C2_pos;
                RE::NiPoint3 new_C2_vector = (upperarm->world.translate - clavicle2->world.translate);
                original_C2_vector.Unitize();
                new_C2_vector.Unitize();
                auto axis = original_C2_vector.Cross(new_C2_vector);
                auto dot = original_C2_vector.Dot(new_C2_vector);

                mutil::Quaternion vec2vec_quat;
                vec2vec_quat.x = axis.x;
                vec2vec_quat.y = axis.y;
                vec2vec_quat.z = axis.z;
                vec2vec_quat.w = dot;

                auto original_C2_local_matrix = original_C2_quat.torotation3();

                RE::NiMatrix3 original_C2_world_matrix;
                for (int x = 0; x < 3; x++) {
                    original_C2_world_matrix.entry[0][x] = original_C2_local_matrix.columns[x][0];
                    original_C2_world_matrix.entry[1][x] = original_C2_local_matrix.columns[x][1];
                    original_C2_world_matrix.entry[2][x] = original_C2_local_matrix.columns[x][2];
                }
                auto b = clavicle2->world.rotate.entry;
                original_C2_world_matrix = shouldernode->world.rotate * original_C2_world_matrix;
                auto vec2vec_rot3 = vec2vec_quat.torotation3();
                auto x_vecn = vec2vec_rot3.columns[0];
                auto y_vecn = vec2vec_rot3.columns[1];
                auto z_vecn = vec2vec_rot3.columns[2];
                RE::NiMatrix3 vec2vec_matrix3;
                for (int x = 0; x < 3; x++) {
                    vec2vec_matrix3.entry[0][x] = vec2vec_rot3.columns[x][0];
                    vec2vec_matrix3.entry[1][x] = vec2vec_rot3.columns[x][1];
                    vec2vec_matrix3.entry[2][x] = vec2vec_rot3.columns[x][2];
                }         

                logger::error("before\n{} {} {}\n{} {} {}\n{} {} {}", b[0][0], b[0][1], b[0][2], b[1][0], b[1][1], b[1][2], b[2][0],
                              b[2][1], b[2][2]);

                clavicle2->world.rotate = vec2vec_matrix3*original_C2_world_matrix;
                clavicle2->local.rotate = clavicle2->parent->world.rotate.Transpose() * (vec2vec_matrix3*original_C2_world_matrix);

                auto a = clavicle2->world.rotate.entry;
                logger::error("after\n{} {} {}\n{} {} {}\n{} {} {}", a[0][0], a[0][1], a[0][2], a[1][0], a[1][1], a[1][2], a[2][0], a[2][1],
                              a[2][2]);
                RE::NiUpdateData data{0.0f, RE::NiUpdateData::Flag::kDirty};
                obj->GetCurrent3D()->Update(data);

        }
        
    }
    

    void GameEventHandler::onLoad() {
        logger::info("onLoad()");
        Hooks::install();
    }

    void GameEventHandler::onPostLoad() {
        logger::info("onPostLoad()");
    }

    void GameEventHandler::onPostPostLoad() {
        if (OriginalActorUpdatePtr == nullptr) {
            {
                DetourTransactionBegin();
                DetourUpdateThread(GetCurrentThread());
                uintptr_t* actor_vtable = (uintptr_t*) RE::VTABLE_Actor[0].address();
                OriginalActorUpdatePtr = (void*) actor_vtable[0xad];

                DetourAttach(&OriginalActorUpdatePtr, ActorUpdateHook);
                DetourTransactionCommit();
            }
            {
                DetourTransactionBegin();
                DetourUpdateThread(GetCurrentThread());
                uintptr_t* player_vtable = (uintptr_t*) RE::VTABLE_PlayerCharacter[0].address();
                OriginalPlayerUpdatePtr = (void*) player_vtable[0xad];

                DetourAttach(&OriginalPlayerUpdatePtr, PlayerUpdateHook);
                DetourTransactionCommit();
            }
            {
                DetourTransactionBegin();
                DetourUpdateThread(GetCurrentThread());
                
                OriginalPlayerUpdate3PPtr = (void*) REL::RelocationID(39446, 40522).address();

                DetourAttach(&OriginalPlayerUpdate3PPtr, PlayerUpdate3PHook);
                DetourTransactionCommit();
            }
            
		
        }
        logger::info("onPostPostLoad()");
    }

    void GameEventHandler::onInputLoaded() {
        logger::info("onInputLoaded()");
    }

    void GameEventHandler::onDataLoaded() {
        logger::info("onDataLoaded()");
    }

    void GameEventHandler::onNewGame() {
        logger::info("onNewGame()");
    }

    void GameEventHandler::onPreLoadGame() {
        logger::info("onPreLoadGame()");
    }

    void GameEventHandler::onPostLoadGame() {
        logger::info("onPostLoadGame()");
    }

    void GameEventHandler::onSaveGame() {
        logger::info("onSaveGame()");
    }

    void GameEventHandler::onDeleteGame() {
        logger::info("onDeleteGame()");
    }
}  // namespace plugin